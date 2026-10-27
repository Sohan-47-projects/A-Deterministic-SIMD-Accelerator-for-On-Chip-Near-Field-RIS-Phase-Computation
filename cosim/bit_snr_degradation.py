
"""
Step 1: Python Bit-Exact Golden Reference & Test-Vector Generator
Architecture: 4-Way SIMD RIS Phase Synthesis Core (Module 3 & Module 4 Baseline)
Format      : Q4.12 signed fixed-point inputs, Q8.24 intermediate products, 
              16-bit BAM phase representation, 2-bit discrete phase outputs.
"""

import math


#  HARDWARE SYSTEM CONSTANTS & PARAMETERS

NUM_ELEMENTS    = 256              # 16x16 RIS unit cells
NUM_LANES       = 4                # 4-way SIMD parallelism
CYCLES_PER_LANE = NUM_ELEMENTS // NUM_LANES  # 64 cycles per frame
ELEMENT_SPACING = 0.005            # d = 5mm (half-wavelength @ 30 GHz)
CARRIER_FREQ_HZ = 30.0e9           # 30 GHz mmWave carrier
C_LIGHT         = 3.0e8            # Speed of light (m/s)
LAMBDA_M        = C_LIGHT / CARRIER_FREQ_HZ  # lambda = 0.01 m (10 mm)

# Fixed-Point Scale Factors
Q_FRAC_BITS     = 12
Q_SCALE         = 1 << Q_FRAC_BITS # 4096

# BAM Scaling Constant K_lambda:
# theta_bam = (R_Q4.12 * K_lambda) mod 65536
# True conversion: theta = (2*pi/lambda) * R
# BAM maps [0, 2*pi) -> [0, 65536) => theta_bam = (65536 / lambda) * R_real
# With R_real = R_Q412 / 4096 => theta_bam = (65536 / (4096 * lambda)) * R_Q412
K_LAMBDA_FLOAT  = (65536.0 / (Q_SCALE * LAMBDA_M))   # = 1600 for lambda = 10 mm, Q4.12
K_LAMBDA_INT    = int(round(K_LAMBDA_FLOAT))  # Scaled integer multiplier


# THEORETICAL QUANTIZATION LOSS ANALYSIS (ASCII Safe)

def evaluate_quantization_loss():
    print("===================================================================")
    print("THEORETICAL PHASE QUANTIZATION EFFICIENCY (eta_q = sinc^2(pi / 2^B))")
    print("===================================================================")
    print(f"{'Bits (B)':<10} | {'States (2^B)':<15} | {'Gain Loss (dB)':<15} | {'Efficiency (eta_q)':<15}")
    print("-------------------------------------------------------------------")
    for B in [1, 2, 3, 4]:
        num_states = 1 << B
        arg = math.pi / num_states
        sinc_val = math.sin(arg) / arg
        eta_q = sinc_val ** 2
        loss_db = 10.0 * math.log10(eta_q)
        print(f"{B:<10} | {num_states:<15} | {loss_db:<15.3f} | {eta_q * 100:<14.2f}%")
    print("-------------------------------------------------------------------\n")


#  FIXED-POINT & BIT-EXACT HARDWARE PRIMITIVES

def float_to_q(val: float) -> int:
    """Converts a float (metres) to a 16-bit signed Q4.12 integer with saturation clamping (range +/-8 m)."""
    raw = int(round(val * Q_SCALE))
    clamped = max(-32768, min(32767, raw))
    return clamped & 0xFFFF  # Return 16-bit two's complement

def q_to_float(val_16b: int) -> float:
    """Converts a 16-bit two's complement integer back to float."""
    signed_val = val_16b if val_16b < 0x8000 else val_16b - 0x10000
    return signed_val / float(Q_SCALE)

def non_restoring_sqrt_32b(D: int) -> int:
    """
    Bit-exact emulator of the 16-stage hardware Non-Restoring Square Root core.
    Input : D (32-bit unsigned integer: sum of squares)
    Output: R (16-bit unsigned integer root: floor(sqrt(D)))
    """
    assert 0 <= D < (1 << 32), "Radicand D must be an unsigned 32-bit integer"
    
    Q = 0
    rem = 0
    
    # Process 2 bits of radicand per stage across 16 pipelined stages
    for stage in range(15, -1, -1):
        two_bits = (D >> (2 * stage)) & 0x03
        rem = (rem << 2) | two_bits
        
        if rem >= 0:
            test_rem = rem - ((Q << 2) | 1)
        else:
            test_rem = rem + ((Q << 2) | 3)
            
        rem = test_rem
        if rem >= 0:
            Q = (Q << 1) | 1
        else:
            Q = (Q << 1) | 0
            
    return Q & 0xFFFF

def hardware_phase_quantizer(R_q88: int, k_lambda: int, B: int = 2) -> int:
    """
    Bit-exact BAM scaling, rounding bias injection, and MSB bit-slice.
    Input : R_q88 (16-bit unsigned distance in Q4.12)
            k_lambda (BAM scaling constant)
            B (quantization bits: 2 for 4 phase states)
    Output: B-bit state index (0 to 2^B - 1)
    """
    
    full_product = R_q88 * k_lambda
    theta_bam = full_product & 0xFFFF  # 16-bit BAM register (mod 2^16, no extra shift)
    
    # Step 2: Inject Half-LSB Rounding Bias
    # For B=2, bias = 1 << (15 - 2) = 1 << 13 = 0x2000
    bias = 1 << (16 - B - 1)
    biased_phase = (theta_bam + bias) & 0xFFFF
    
    # Step 3: Extract Top B bits
    state_index = (biased_phase >> (16 - B)) & ((1 << B) - 1)
    return state_index


# 4. ELEMENT COORDINATE GEOMETRY GENERATOR

def generate_ris_element_coords():
    """
    Generates a 16x16 planar grid centered at the origin (Z=0 plane).
    Returns coordinates partitioned into 4 SIMD lane arrays.
    """
    coords = []
    for r in range(16):
        for c in range(16):
            # Center the array at (0, 0, 0)
            x_m = (c - 7.5) * ELEMENT_SPACING
            y_m = (r - 7.5) * ELEMENT_SPACING
            z_m = 0.0
            coords.append((x_m, y_m, z_m))
            
    # Partition into 4 lanes: Lane i gets indices [i, i+4, i+8, ...]
    lanes = [[] for _ in range(NUM_LANES)]
    for idx, pt in enumerate(coords):
        lanes[idx % NUM_LANES].append(pt)
        
    return coords, lanes


# BIT-EXACT VERIFICATION RUNNER & TEST VECTOR WRITER

def run_golden_simulation():
    evaluate_quantization_loss()
    
    # Define Target Test Cases: (X_t, Y_t, Z_t) in meters
    test_cases = [
        {"name": "Boresight Near-Field", "pos": (0.00, 0.00, 0.50)},
        {"name": "Off-Axis Tracking 1",  "pos": (0.15, 0.10, 0.80)},
        {"name": "Deep Angle Steering",  "pos": (-0.25, 0.30, 1.00)},
    ]
    
    all_coords, lane_coords = generate_ris_element_coords()
    
    with open("stimulus_in.hex", "w", encoding="utf-8") as f_stim, open("expected_out.hex", "w", encoding="utf-8") as f_exp:
        for t_idx, tc in enumerate(test_cases):
            tx, ty, tz = tc["pos"]
            tx_q88 = float_to_q(tx)
            ty_q88 = float_to_q(ty)
            tz_q88 = float_to_q(tz)
            
            print(f"Generating Case {t_idx+1}: {tc['name']} -> Target ({tx:.2f}, {ty:.2f}, {tz:.2f}) m")
            
            # Write Stimulus Header: Target Coordinates & Lambda Constant
            # Format: TX[15:0] TY[15:0] TZ[15:0] K_LAMBDA[15:0]
            f_stim.write(f"{tx_q88:04X}_{ty_q88:04X}_{tz_q88:04X}_{K_LAMBDA_INT:04X}\n")
            
            frame_2bit_states = []
            
            # Stream across 64 cycles for the 4 SIMD lanes
            for cyc in range(CYCLES_PER_LANE):
                lane_outputs = []
                for lane in range(NUM_LANES):
                    elem_idx = cyc * NUM_LANES + lane
                    ex, ey, ez = all_coords[elem_idx]
                    
                    ex_q88 = float_to_q(ex)
                    ey_q88 = float_to_q(ey)
                    ez_q88 = float_to_q(ez)
                    
                    # 1. Delta Calculations (16-bit signed Q4.12)
                    dx = (tx_q88 - ex_q88) & 0xFFFF
                    dy = (ty_q88 - ey_q88) & 0xFFFF
                    dz = (tz_q88 - ez_q88) & 0xFFFF
                    
                    # Convert to signed for arithmetic multiplication
                    dx_s = dx if dx < 0x8000 else dx - 0x10000
                    dy_s = dy if dy < 0x8000 else dy - 0x10000
                    dz_s = dz if dz < 0x8000 else dz - 0x10000
                    
                    # 2. DSP48 Sum of Squares (Q8.24 format -> 32-bit unsigned)
                    D_q8_24 = (dx_s * dx_s) + (dy_s * dy_s) + (dz_s * dz_s)
                    
                    # 3. Non-Restoring Integer Square Root (Produces Q4.12 root)
                    R_q88 = non_restoring_sqrt_32b(D_q8_24)
                    
                    # 4. BAM Quantizer (Produces 2-bit state)
                    state_2b = hardware_phase_quantizer(R_q88, K_LAMBDA_INT, B=2)
                    lane_outputs.append(state_2b)
                    frame_2bit_states.append(state_2b)
                    
            # 5. Pack the 256 2-bit states into 16 32-bit words for AXI read registers
            # Each 32-bit word holds 16 elements (16 * 2 bits = 32 bits)
            for w in range(16):
                word_val = 0
                for bit_pos in range(16):
                    elem_num = w * 16 + bit_pos
                    state = frame_2bit_states[elem_num]
                    word_val |= (state << (bit_pos * 2))
                f_exp.write(f"{word_val:08X}\n")
                
    print("\n[SUCCESS] Generated:")
    print("  -> stimulus_in.hex  (Input target coordinates & configuration)")
    print("  -> expected_out.hex (Bit-exact 16x 32-bit packed AXI output registers)")

if __name__ == "__main__":
    run_golden_simulation()