"""Monte Carlo for the re-acquisition FSM: 30 dB blockage, position measurements rejected (tracker coasts)
while the link is blocked. Phase states come from the bit-exact golden model (equal to the RTL on every
word checked). A run 'executes all four stages' if EXPAND, SWEEP and RESET are all visited in the
blockage/recovery interval and the FSM is back in TRACKING_LOCK afterwards."""
import numpy as np
from ipc_closed_loop_fixed import (PredictiveEKF, ReacquisitionFSM, golden_phase_matrix, evaluate_array_gain)
def run(seed, att=30.0):
    r=np.random.default_rng(seed); dt=.02; N=60; t=np.arange(N)*dt
    Ax,Ay,Az=r.uniform(.15,.22),r.uniform(.10,.16),r.uniform(.05,.10); px,py,pz=r.uniform(0,2*np.pi,3)
    P=np.stack([Ax*np.sin(2*np.pi*t+px),Ay*np.sin(4*np.pi*t+py),0.85+Az*np.cos(np.pi*t+pz)],1)
    on=int(r.integers(10,41)); off=on+int(r.integers(6,13))-1
    ekf=PredictiveEKF(init_pos=P[0].copy()); fsm=ReacquisitionFSM(); prev=25.0; st=[]; G=[]
    for k in range(N):
        blocked=on<=k<=off
        _,pp = ekf.predict_only() if blocked else ekf.step(P[k]+r.normal(0,.012,3))
        s,steer=fsm.update(prev,pp,ekf)
        rec=evaluate_array_gain(P[k],golden_phase_matrix(steer))-(att if blocked else 0.0)
        prev=rec; st.append(s); G.append(rec)
    return np.array(st),np.array(G),on,off
if __name__=="__main__":
    M=500; all4=0; sweep=0; first_sweep=[]; lock_back=[]; outside=[]
    for s in range(M):
        st,G,on,off=run(5000+s); w=st[on:off+6]
        sweep+=bool((w==2).any())
        ok=bool((w==1).any() and (w==2).any() and (w==3).any() and st[min(off+6,59):].max()==0)
        all4+=ok
        k2=np.where(st[on:off+6]==2)[0]
        if len(k2): first_sweep.append(k2[0])
        lk=np.where(st[off+1:]==0)[0]; lock_back.append(lk[0]+1 if len(lk) else 99)
        o=np.ones(60,bool); o[on:off+6]=False; outside.append(np.mean(st[o]!=0))
    print(f"{M} runs, 30 dB, measurements rejected during blockage")
    print(f"  all four stages executed: {all4}/{M} ({100*all4/M:.1f}%)   reached CODEBOOK_SWEEP: {sweep}/{M} ({100*sweep/M:.1f}%)")
    print(f"  CODEBOOK_SWEEP first reached (frames after onset): median {np.median(first_sweep):.0f}, IQR {np.percentile(first_sweep,25):.0f}-{np.percentile(first_sweep,75):.0f}")
    print(f"  TRACKING_LOCK regained (frames after last blocked frame): median {np.median(lock_back):.0f}, max {max(lock_back)}")
    print(f"  fraction of frames outside TRACKING_LOCK, away from the blockage/recovery interval: {100*np.mean(outside):.2f}%")
