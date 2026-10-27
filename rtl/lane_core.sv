`timescale 1ns / 1ps

module lane_core #(
    parameter int B = 2
)(
    input  logic         clk,
    input  logic         rst_n,
    
    // Control / Strobe
    input  logic         valid_in,
    input  logic [7:0]   elem_idx_in,   // Element index (0 to 255)
    
    // Target Coordinates (Q8.8 signed)
    input  logic signed [15:0] target_x,
    input  logic signed [15:0] target_y,
    input  logic signed [15:0] target_z,
    
    // Local Element Coordinates (Q8.8 signed from ROM)
    input  logic signed [15:0] elem_x,
    input  logic signed [15:0] elem_y,
    input  logic signed [15:0] elem_z,
    
    // Configuration
    input  logic [15:0]  k_lambda,      // BAM phase scaling constant
    
    // Outputs
    output logic         valid_out,
    output logic [7:0]   elem_idx_out,  // Aligned element index
    output logic [B-1:0] state_out      // B-bit quantized state
);

  
    // LOCAL DELTA COMPUTATION (Combinatorial into DSP stage 1 latches)
    
    logic signed [15:0] dx;
    logic signed [15:0] dy;
    logic signed [15:0] dz;
    
    assign dx = target_x - elem_x;
    assign dy = target_y - elem_y;
    assign dz = target_z - elem_z;

   
    // PIPELINE INTERCONNECT SIGNALS
    
    logic        sq_valid_out;
    logic [31:0] sq_D_out;
    
    logic        sqrt_valid_out;
    logic [15:0] sqrt_Q_out;

    
    // 1. DSP SQUARER CASCADE (Latency: 4 cycles)
   
    dsp_squarer_cascade u_squarer (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .dx_in     (dx),
        .dy_in     (dy),
        .dz_in     (dz),
        .valid_out (sq_valid_out),
        .D_out     (sq_D_out)
    );

   
    // 2. NON-RESTORING SQUARE ROOT (Latency: 16 cycles)
    
    non_restoring_sqrt u_sqrt (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (sq_valid_out),
        .D_in      (sq_D_out),
        .valid_out (sqrt_valid_out),
        .Q_out     (sqrt_Q_out)
    );

    
    // 3. PHASE QUANTIZER (Latency: 2 cycles)
   
    phase_quantizer #(
        .B (B)
    ) u_quantizer (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (sqrt_valid_out),
        .R_in      (sqrt_Q_out),
        .k_lambda  (k_lambda),
        .valid_out (valid_out),
        .state_out (state_out)
    );

    
    // METADATA PIPELINE: 22-CYCLE SHIFT REGISTER FOR ELEMENT INDEX
    // No asynchronous reset on data taps 100% SRL32E primitive inference
  
    logic [7:0] idx_pipe [0:21];
    
    always_ff @(posedge clk) begin
        idx_pipe[0] <= elem_idx_in;
        for (int i = 1; i < 22; i++) begin
            idx_pipe[i] <= idx_pipe[i-1];
        end
    end

    assign elem_idx_out = idx_pipe[21];

endmodule