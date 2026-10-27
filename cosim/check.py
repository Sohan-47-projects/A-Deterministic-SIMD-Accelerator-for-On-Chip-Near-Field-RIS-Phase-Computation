import subprocess
import sys

def run_command(cmd, desc):
    print(f"\n[*] =========================================")
    print(f"[*] Running: {desc}")
    print(f"[*] Command: {cmd}")
    print(f"[*] =========================================")
    
    # Run with direct stdout/stderr passthrough so output appears in real time
    ret = subprocess.call(cmd, shell=True)
    if ret != 0:
        print(f"\n[!] FAILED: {desc} (Exit code: {ret})")
        sys.exit(ret)

# 1. Run Vector Generator
run_command("python bit_snr_degradation.py", "Golden Reference Vector Generation")

# 2. Compile RTL via Icarus Verilog
run_command(
    "iverilog -g2012 -o sim.vvp non_restoring_sqrt.sv dsp_squarer_cascade.sv "
    "phase_quantizer.sv lane_core.sv ris_core_top.sv axi4_lite_slave.sv tb_top.sv",
    "Icarus RTL Compilation"
)

# 3. Execute Simulation
run_command("vvp sim.vvp", "HDL Simulation Execution")

print("\n[+] Verification suite executed successfully.")