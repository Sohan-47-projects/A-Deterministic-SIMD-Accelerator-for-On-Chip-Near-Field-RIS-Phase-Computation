
# A Deterministic SIMD Accelerator for On-Chip Near-Field RIS Phase Computation

[![Hardware](https://img.shields.io/badge/Hardware-Xilinx%20Zynq--7020-red)](#fpga-implementation--post-route-results)
[![HDL](https://img.shields.io/badge/HDL-SystemVerilog-blue)](#verification--closed-loop-co-simulation)
[![Co-Simulation](https://img.shields.io/badge/Co--Sim-Python%20%2B%20Icarus%20Verilog-green)](#verification--closed-loop-co-simulation)
[![Target](https://img.shields.io/badge/Status-IEEE%20ESL%20Submission-orange)](#citation)

> **GitHub "About" Description:**  
> A deterministic 4-lane SIMD FPGA accelerator for codebook-free, real-time near-field RIS beam steering ($16\times16$ array at 30 GHz). Computes 256-element 2-bit phase profiles in 87 cycles (580 ns at 150 MHz) via a Q4.12 non-restoring square-root and BAM datapath.

---

## Overview

Near-field Reconfigurable Intelligent Surface (RIS) beam focusing requires computing an exact spherical-wavefront distance per element rather than applying a uniform far-field angular gradient. Because this calculation must complete within the channel coherence time, existing embedded RIS controllers typically bypass real-time geometry evaluation using precomputed codebooks or machine learning inference.

This project implements a **deterministic 4-lane SIMD hardware accelerator** that computes the exact near-field Euclidean range and 2-bit quantized phase for every element directly from 3D target coordinates at run time—requiring **zero stored phase codebooks or trained models**. 

### Key Contributions
* **Codebook-Free 4-Lane SIMD Datapath:** Exploits element-wise spatial independence to evaluate a $16\times16$ ($N=256$) planar RIS at 30 GHz ($\lambda = 10\text{ mm}$, $5\text{ mm}$ spacing). Update latency is strictly a function of array size and lane count, completely independent of target position.
* **High-Precision Q4.12 Fixed-Point Pipeline:** Pairs DSP48 coordinate squaring with a multiplierless, data-independent 16-stage non-restoring square-root recurrence and Binary Angular Measurement (BAM) quantization, limiting mean array gain loss to just **0.01 dB** relative to an ideal floating-point range reference.
* **End-to-End Hardware-in-the-Loop Validation:** Verified bit-exactly across 212,992 phase states and 2.13M square-root radicands, validated semantically via 3D physical array-factor modeling, and tested inside a lockstep TCP/IP Python-to-RTL closed-loop Kalman filter tracking and 30 dB blockage-recovery system.

---

## System Architecture

For a steering target at $\mathbf{p}_t$ and the $i$-th RIS element at $\mathbf{p}_i$, the required phase shift is:

$$\theta_i = \frac{2\pi}{\lambda} R_i, \quad R_i = \sqrt{\Delta x_i^2 + \Delta y_i^2 + \Delta z_i^2} = \|\mathbf{p}_t - \mathbf{p}_i\|_2$$

```text
+-------------------+   target    +-------------------+   start    +-------------------+
|     Host CPU      | ----------> |  AXI4-Lite Slave  | ---------> |   Dispatch Ctrl   |
|    (KF + FSM)     |             |      + CSRs       |            |   + 4 ROM Banks   |
+-------------------+             +-------------------+            +-------------------+
          ^                                                                  |
          | readback                                                         | coords
          |                                                                  v
+-------------------+    2-b      +----------------------------------------------------+
|  512-b Phase Reg  | <---------  | 4 SIMD Lanes (22 Cycles / Element, II = 1):        |
|  File (16x32-bit) |   states    |  [1] DSP48 Cascade Squarer (4 cycles)              |
+-------------------+             |  [2] Non-Restoring Sqrt Recurrence (16 cycles)     |
          |                       |  [3] BAM Scaling + 2-bit Quantizer (2 cycles)      |
          | phases                +----------------------------------------------------+
          v
+-------------------+
| PIN-Diode Control |
+-------------------+
```

### 1. Fixed-Point Format & 22-Cycle Lane Pipeline
* **Number Representation:** Target and element coordinates use signed **Q4.12** fixed-point meters ($\text{LSB} = 0.24\text{ mm} = \lambda/41$, span $\pm 8\text{ m}$). Squared distance $D$ is unsigned **Q8.24** (32-bit), and extracted range $R$ is unsigned **Q4.12** (16-bit), yielding $8.8^\circ$ of phase resolution per range LSB.
* **Stage 1 — Squarer (4 Cycles):** A DSP48-cascade unit computes $D = \Delta x^2 + \Delta y^2 + \Delta z^2$.
* **Stage 2 — Non-Restoring Square Root (16 Cycles):** Consumes two radicand bits and produces one root bit per stage ($R = \sqrt{D}$) with an initiation interval $\text{II} = 1$ and **0 DSP48s**, avoiding the multiplier overhead of Newton-Raphson/Taylor schemes and the scale-factor correction of CORDIC.
* **Stage 3 — BAM Quantizer (2 Cycles):** Accumulates phase in a 16-bit BAM register (exploiting natural unsigned overflow modulo $2\pi$), applies a half-LSB rounding bias, and extracts the top $B = 2$ bits ($\Phi \in \{0, \pi/2, \pi, 3\pi/2\}$).

### 2. Deterministic SIMD Scheduling & Host Interface
* **Conflict-Free Dispatch:** Four dedicated ROM banks store static element geometry (not phase states), feeding one coordinate tuple per lane per cycle.
* **Latency Model:** With pipeline depth $L_{\text{pipe}} = 22$, total cycle latency for $N$ elements across $P$ lanes is strictly deterministic:

$$L(N, P) = \left\lceil \frac{N}{P} \right\rceil + L_{\text{pipe}} + 1$$

  For $N = 256$ and $P = 4$, a complete frame update takes $64 + 22 + 1 = \mathbf{87\text{ cycles}}$ ($\mathbf{580\text{ ns}}$ at $150\text{ MHz}$).
* **AXI4-Lite Control:** An output packer writes 2-bit states into a 512-bit phase register file exposed as sixteen 32-bit registers over AXI4-Lite. The register file drives the RIS PIN diodes directly while supporting host readback for verification.

---

## FPGA Implementation & Post-Route Results

Synthesized, placed, and routed in **AMD Vivado** targeting a **Xilinx Zynq-7020 (`xc7z020clg400-1`)** with a $150\text{ MHz}$ ($6.667\text{ ns}$) clock constraint.

### Sequential ($P=1$) vs. 4-Way SIMD ($P=4$) Comparison

| Metric | Single-Lane ($P=1$) | 4-Lane SIMD ($P=4$) | Scaling Ratio / Utilization |
| :--- | :---: | :---: | :---: |
| **Frame Cycles (to `done`)** | 279 cycles | **87 cycles** | **3.21× speedup** |
| **Frame Update Latency** | 1860 ns | **580 ns** | **0.35% of 167-µs coherence bound** |
| **Slice LUTs** | 1,212 | **2,642** | 2.18× ($\approx 5.0\%$ of Zynq-7020) |
| **Slice Flip-Flops (FFs)** | 1,196 | **3,086** | 2.58× ($\approx 2.9\%$ of Zynq-7020) |
| **DSP48 Slices** | 5 | **20** | 4.00× ($\approx 9.1\%$ of Zynq-7020) |
| **Block RAM (BRAM)** | 0 | **0** | Distributed LUT-ROM only |
| **Worst Negative Slack (WNS)** | +0.208 ns | **+0.075 ns** | Closes timing at 150 MHz |
| **Dynamic / Static Power** | — | **126 mW / 104 mW** | $\approx 73\text{ nJ}$ dynamic energy per update |

> **200 MHz Deeper-Pipeline Variant:** Registering every stage of the square-root recurrence increases $L_{\text{pipe}}$ from $22 \rightarrow 37$ cycles (frame update $87 \rightarrow 102$ cycles), successfully closing timing at **200 MHz** ($\text{WNS} = +0.084\text{ ns}$) and reducing absolute update latency from **580 ns to 510 ns**.

---

## Numerical Precision Analysis (Q4.12 vs. Q8.8)

Evaluated across 400 targets per spatial regime against an ideal floating-point range 2-bit reference. Both formats share identical RTL width (16-bit coordinates), differing only in ROM contents and the BAM scaling constant $K_\lambda$:

| Metric | Q4.12 Format (Selected) | Q8.8 Format (Baseline) |
| :--- | :---: | :---: |
| **Range LSB / Phase per LSB** | $0.24\text{ mm} / 8.8^\circ$ | $3.9\text{ mm} / 140.6^\circ$ |
| **Max Phase Error (Front / Beside)** | $\mathbf{7^\circ / 9^\circ}$ (median $2^\circ$) | $122^\circ / 165^\circ$ |
| **Mean Array Gain Loss (Front / Beside)** | **0.01 dB / 0.01 dB** | 2.4 dB / 3.0 dB |
| **Worst-Target Gain Loss** | **0.3 dB** | 4.8 dB |

---

## Verification & Closed-Loop Co-Simulation

### 1. Bit-Exact Golden Model & Corner Verification
* **Golden Reference:** A bit-exact Python model replicates Q4.12 saturation, the 16-stage non-restoring recurrence, and BAM quantization.
* **Exhaustive Vector Regression:** The SystemVerilog RTL matched the golden model across **832 target configurations** with varied $K_\lambda$ (216 saturation/corner cases, 16 on-element, 600 random; **212,992 total phase states**).
* **Standalone Square-Root Verification:** Matched exact integer roots across **2.13M radicands** (all perfect squares, predecessors, extremes, and back-to-back/gapped random streams).

### 2. Semantic Array-Factor Validation
To prevent common-mode specification bugs between the golden model and RTL, every returned phase matrix is evaluated against a physical 3D near-field beam-pattern metric incorporating an element obliquity factor $w_{m,n} = (\hat{z}/R_{m,n})^q$ with $q = 1.2$ ($8.3\text{ dBi}$ patch element pattern):

$$S = 10\log_{10} \left| \sum_{m,n=1}^{16} w_{m,n} e^{j(\Phi_{m,n} - k\Delta d_{m,n})} \right|^2$$

* **Theoretical Cophasal Ceiling:** $10\log_{10}(N^2) \approx 48.16\text{ dB}$ (with theoretical 2-bit quantization loss $\eta_q = \text{sinc}^2(\pi/2^B) \approx 0.91\text{ dB}$).
* **Achieved Q4.12 Static Gain:** **47.07 dB** across 500 trajectory volume targets—only **0.007 dB** below the ideal floating-point 2-bit profile.

### 3. Closed-Loop Tracking & Blockage Co-Simulation
A lockstep TCP socket server (`ipc_hw_runner.py`) bridges a Python **Constant-Velocity Kalman Filter (KF)** and **4-State Recovery FSM** (`TRACKING_LOCK`, `COVARIANCE_EXPAND`, `CODEBOOK_SWEEP`, `LINK_RESET`) directly to the compiled SystemVerilog RTL running inside **Icarus Verilog**:
* **3D Mobility Trajectory:** 60-epoch trajectory across a $44\times32\times20\text{ cm}$ volume at $z \approx 0.85\text{ m}$ (peak velocity $2.5\text{ m/s}$, observation noise $\sigma_r = 12\text{ mm}$, tracking error $29\text{ mm}$ RMS).
* **Blockage Recovery:** Under an injected **30 dB obstruction** (frames 22–30), the system maintains a mean closed-loop array gain of **46.0 dB** outside the blockage window and autonomously sweeps and re-acquires lock.
* **Monte Carlo Robustness:** Across **500 randomized Monte Carlo trials** (varying trajectory kinematics, noise seeds, blockage onset, and duration), the controller recovered lock within **5 frames** of blockage clearance in **100% of trials**.

<img width="2420" height="726" alt="cosim_results" src="https://github.com/user-attachments/assets/f722e741-0837-44aa-9e79-39a880a7a316" />

### 4. Channel Coherence & Scalability Context
Under the Jakes fading model ($T_c \approx 9/(16\pi f_D)$) at $30\text{ GHz}$, a rapid mobile velocity of $v \approx 10.7\text{ m/s}$ yields a coherence time of $T_c = 167\text{ }\mu\text{s}$.
* The accelerator's **580 ns** update consumes just **0.35%** of the coherence window (compared to $15\text{--}68\text{ }\mu\text{s}$ simulated on a 667 MHz ARM Cortex-A9), freeing the host CPU entirely for state estimation.
* For larger apertures, the 4-lane architecture scales deterministically at 1 cycle per 4 elements, projecting **1.9 µs** for $N=1024$ and **3.6 µs** for $N=2048$.

---


## Author

**Sohan Bag**  
Department of Electronics and Communication Engineering  
MCKV Institute of Engineering, Howrah 711204, India  
ORCID: [0009-0000-2781-458X](https://orcid.org/0009-0000-2781-458X) | Email: `bag.sohan2006@gmail.com`
