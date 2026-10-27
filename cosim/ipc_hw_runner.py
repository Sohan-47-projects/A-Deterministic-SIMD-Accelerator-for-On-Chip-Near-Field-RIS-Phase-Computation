import socket
import struct
import subprocess
import os

HOST = "127.0.0.1"
PORT = 65432

def run_sv_hardware(tx_q, ty_q, tz_q, k_lam):
    # 1. Write stimulus vector for Verilog
    with open("hdl_input.hex", "w") as f:
        f.write(f"{tx_q & 0xFFFF:04x} {ty_q & 0xFFFF:04x} {tz_q & 0xFFFF:04x} {k_lam & 0xFFFF:04x}\n")

    # 2. Invoke compiled Icarus Verilog core
    subprocess.run("vvp cosim_core.vvp", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 3. Read back 16 packed 32-bit registers
    regs = []
    with open("hdl_output.hex", "r") as f:
        for line in f:
            if line.strip():
                regs.append(int(line.strip(), 16))
    return regs

def main():
    print("=" * 65)
    print("  PHASE 1: HARDWARE IPC RUNNER (TCP LOCAL BRIDGE)")
    print("=" * 65)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[*] Listening on {HOST}:{PORT}. Waiting for Python master to connect...")

        conn, addr = s.accept()
        with conn:
            print(f"[*] Master connected from {addr}. Starting lock-step loop...")
            while True:
                data = conn.recv(16)
                if not data or len(data) < 16:
                    break

                # Unpack command: step_id (I), tx_q (h), ty_q (h), tz_q (h), k_lam (H), cmd (I)
                step_id, tx_q, ty_q, tz_q, k_lam, cmd = struct.unpack("<IhhhHI", data)

                if cmd == 2: # SHUTDOWN
                    print("[*] Shutdown signal received. Exiting.")
                    break

                # Execute SystemVerilog simulation
                regs = run_sv_hardware(tx_q, ty_q, tz_q, k_lam)

                # Send back: cycles (I) + 16 registers (16I) = 68 bytes
                response_payload = struct.pack("<I16I", 86, *regs)
                conn.sendall(response_payload)

    print("[*] Hardware runner closed cleanly.")

if __name__ == "__main__":
    main()