"""Writes rom_lane0..3.hex (Q4.12 element coordinates, 3 x 16-bit per line: X Y Z) for the unchanged RTL.
Run next to bit_snr_degradation.py (Q4.12 version). Lane i holds elements i, i+4, i+8, ... of the row-major 16x16 grid."""
import bit_snr_degradation as g
allc,_=g.generate_ris_element_coords()
for lane in range(4):
    with open(f"rom_lane{lane}.hex","w") as f:
        for cyc in range(64):
            x,y,z=allc[cyc*4+lane]; f.write(f"{g.float_to_q(x):04x}{g.float_to_q(y):04x}{g.float_to_q(z):04x}\n")
