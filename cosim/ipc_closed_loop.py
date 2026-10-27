"""Patched closed-loop master. Changes vs original:
 1. K_lambda imported from the golden model (1600 for Q4.12) instead of (65536/lambda)&0xFFFF == 0
 2. per-frame RTL-vs-golden phase-matrix check (semantic/bit-exact guard)
 3. COVARIANCE_EXPAND now actually inflates Q (x10), restored on exit
 4. hw latency converted at 150 MHz (not 200)
 5. blockage depth is a parameter (default 30 dB); position measurements are rejected
    (tracker coasts) while the link is blocked; headless plotting
"""
import socket, struct, time, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from bit_snr_degradation import K_LAMBDA_INT, Q_FRAC_BITS

HOST, PORT = "127.0.0.1", 65432
LAM = 0.01
KK = 2*np.pi/LAM
_g = np.linspace(-7.5, 7.5, 16)*0.005
EX, EY = np.meshgrid(_g, _g)
PH = np.array([0, np.pi/2, np.pi, 3*np.pi/2])

def _isqrt(D):
    r = np.floor(np.sqrt(D.astype(np.float64))).astype(np.int64)
    r -= (r*r > D); r += ((r+1)*(r+1) <= D); return r

def golden_phase_matrix(t, K=K_LAMBDA_INT, frac=Q_FRAC_BITS):
    s = 1 << frac
    tq = [int(round(v*s)) for v in t]
    eqx = np.rint(EX*s).astype(np.int64); eqy = np.rint(EY*s).astype(np.int64)
    D = ((tq[0]-eqx)**2 + (tq[1]-eqy)**2 + tq[2]**2) & 0xFFFFFFFF
    th = (_isqrt(D)*K) & 0xFFFF
    return ((th + 0x2000) & 0xFFFF) >> 14

def evaluate_array_gain(true_pos, pm):
    R = np.sqrt((true_pos[0]-EX)**2 + (true_pos[1]-EY)**2 + true_pos[2]**2)
    w = np.maximum(0, true_pos[2]/R)**1.2
    f = np.sum(w*np.exp(1j*(PH[pm] - KK*R)))
    return 10*np.log10(abs(f)**2 + 1e-12)

class RISHardwareBridge:
    def __init__(self, wavelength=LAM):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.k_lambda = int(round(65536.0/((1<<Q_FRAC_BITS)*wavelength)))
        assert self.k_lambda == K_LAMBDA_INT and 0 < self.k_lambda <= 0xFFFF
        self.mismatch_frames = 0
    def step(self, step_id, t):
        q = [int(round(v*float(1<<Q_FRAC_BITS))) for v in t]
        self.sock.sendall(struct.pack("<IhhhHI", step_id, q[0], q[1], q[2], self.k_lambda, 1))
        raw = b""
        while len(raw) < 68:
            c = self.sock.recv(68-len(raw))
            if not c: raise ConnectionError("runner disconnected")
            raw += c
        u = struct.unpack("<I16I", raw); cycles, regs = u[0], u[1:]
        pm = np.zeros((16,16), dtype=np.uint8)
        for w, word in enumerate(regs):
            for e in range(16): pm[w, e] = (word >> (2*e)) & 3
        if not np.array_equal(pm, golden_phase_matrix(t)): self.mismatch_frames += 1
        return pm, (cycles+1)/150e6*1e6, regs   # runner reports 86 compute cycles; done asserts on cycle 87
    def shutdown(self):
        self.sock.sendall(struct.pack("<IhhhHI", 0,0,0,0,0,2)); self.sock.close()

class MobileTrajectory:
    def __init__(self, steps=60, dt=0.02):
        self.t = np.linspace(0, steps*dt, steps)
        self.P = np.stack([0.22*np.sin(2*np.pi*self.t), 0.16*np.sin(4*np.pi*self.t),
                           0.85+0.10*np.cos(np.pi*self.t)], 1)
    def get_pos(self, k): return self.P[k]
    def get_meas(self, k): return self.P[k] + np.random.normal(0, 0.012, 3)

class PredictiveEKF:   # linear CV Kalman filter
    def __init__(self, dt=0.02, init_pos=np.array([0.0,0.0,0.85])):
        self.dt=dt; self.x=np.zeros(6); self.x[:3]=init_pos
        self.A=np.eye(6); self.A[0,3]=self.A[1,4]=self.A[2,5]=dt
        self.H=np.zeros((3,6)); self.H[0,0]=self.H[1,1]=self.H[2,2]=1.0
        self.P=np.eye(6)*0.1; self.Q=np.eye(6)*1e-4; self.Q[3:,3:]*=10.0
        self.R=np.eye(3)*(0.012**2); self.q_scale=1.0
    def step(self, z):
        xp=self.A@self.x; Pp=self.A@self.P@self.A.T+self.Q*self.q_scale
        S=self.H@Pp@self.H.T+self.R; K=Pp@self.H.T@np.linalg.inv(S)
        self.x=xp+K@(z-self.H@xp); self.P=(np.eye(6)-K@self.H)@Pp
        return self.x[:3], (self.A@self.x)[:3]
    def predict_only(self):   # measurement rejected: coast on the constant-velocity model
        self.x=self.A@self.x; self.P=self.A@self.P@self.A.T+self.Q*self.q_scale
        return self.x[:3], (self.A@self.x)[:3]
    def reset_covariance(self, a): self.x[:3]=a; self.x[3:]=0; self.P=np.eye(6)*0.05

class ReacquisitionFSM:
    LOCK,EXPAND,SWEEP,RESET = 0,1,2,3
    NAMES=["TRACKING_LOCK","COVARIANCE_EXPAND","CODEBOOK_SWEEP","LINK_RESET"]
    def __init__(self, inflate=True):
        self.state=0; self.i=0; self.inflate=inflate
        self.codebook=[np.array(v) for v in
            [(-.18,0,.85),(.18,0,.85),(0,-.12,.85),(0,.12,.85),(0,0,.85)]]
    def update(self, g, fut, ekf):
        tgt=fut.copy()
        if self.state==0:
            if g<10: self.state=1
        elif self.state==1:
            if g>=10: self.state=0
            elif g<3: self.state=2; self.i=0
        elif self.state==2:
            tgt=self.codebook[self.i]; self.i=(self.i+1)%5
            if g>=3: self.state=3
        elif self.state==3:
            ekf.reset_covariance(tgt); self.state=0
        ekf.q_scale = 10.0 if (self.inflate and self.state==1) else 1.0
        return self.state, tgt

def main(block_db=30.0, on=22, off=30, seed=2026):
    np.random.seed(seed)
    traj=MobileTrajectory(); ekf=PredictiveEKF(init_pos=traj.get_pos(0).copy()); fsm=ReacquisitionFSM()
    br=RISHardwareBridge(); prev=25.0
    H=dict(true=[],pred=[],steer=[],gain=[],fsm=[],lat=[],nz=[])
    for k in range(60):
        pt=traj.get_pos(k); blocked = on<=k<=off
        # blockage attenuates the link and the RF-based position measurement is rejected: tracker coasts
        _,pp = ekf.predict_only() if blocked else ekf.step(traj.get_meas(k))
        att=block_db if blocked else 0.0
        s,steer=fsm.update(prev,pp,ekf)
        pm,lat,regs=br.step(k,steer)
        rec=evaluate_array_gain(pt,pm)-att; prev=rec
        for key,v in zip(H,[pt,pp,steer,rec,s,lat,sum(r!=0 for r in regs)]): H[key].append(v)
    br.shutdown()
    H={k:np.array(v) for k,v in H.items()}
    np.savez("closed_loop_results.npz", **H)
    print(f"RTL-vs-golden mismatching frames: {br.mismatch_frames}/60 | K sent = {br.k_lambda} (0x{br.k_lambda:04x})")
    print(f"non-zero output words per frame: min={H['nz'].min()} max={H['nz'].max()} (of 16)")
    print(f"hw latency used in log: {H['lat'][0]:.3f} us")
    print("frame: FSM | gain(dB)")
    prevs=-1
    for k in range(60):
        if H['fsm'][k]!=prevs or on-1<=k<=off+4:
            print(f"  {k:2d}: {ReacquisitionFSM.NAMES[H['fsm'][k]]:<18s} {H['gain'][k]:6.1f}")
        prevs=H['fsm'][k]
    ta=np.arange(60)*0.02
    fig=plt.figure(figsize=(14,9))
    a=fig.add_subplot(2,2,1,projection='3d')
    a.plot(*H['true'].T,'k-',lw=2,label='Ground truth'); a.plot(*H['pred'].T,'b--',label='KF prediction')
    sw=H['fsm']==2; a.scatter(*H['steer'][sw].T,color='red',s=30,label='Sweep points'); a.legend(fontsize=8)
    a.set_title("3D Trajectory Tracking"); 
    a=fig.add_subplot(2,2,2); a.step(ta,H['fsm'],'m-',where='post'); a.set_yticks(range(4)); a.set_yticklabels(ReacquisitionFSM.NAMES)
    a.axvspan(on*.02,off*.02,color='red',alpha=.15,label=f'{block_db:.0f} dB blockage'); a.legend(); a.set_title("Re-acquisition FSM"); a.set_xlabel("Time (s)")
    a=fig.add_subplot(2,2,3); a.plot(ta,H['gain'],'g-'); a.axvspan(on*.02,off*.02,color='red',alpha=.15)
    a.axhline(10,ls=':',c='gray'); a.axhline(3,ls=':',c='gray'); a.set_title("Received gain (dB), RTL phases"); a.set_xlabel("Time (s)")
    a=fig.add_subplot(2,2,4); a.plot(ta,1000*np.linalg.norm(H['pred']-H['true'],axis=1),'b-'); a.set_title("KF one-step prediction error (mm)"); a.set_xlabel("Time (s)")
    plt.tight_layout(); plt.savefig("closed_loop_fixed.png",dpi=200)
if __name__=="__main__": main()
