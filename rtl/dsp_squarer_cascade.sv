`timescale 1ns / 1ps

module dsp_squarer_cascade (
    input  logic               clk,
    input  logic               rst_n,
    
    // Control / Strobe
    input  logic               valid_in,
    
    // 16-bit Signed Fixed-Point Coordinate Deltas (Q8.8)
    input  logic signed [15:0] dx_in,
    input  logic signed [15:0] dy_in,
    input  logic signed [15:0] dz_in,
    
    // Outputs
    output logic               valid_out,
    output logic        [31:0] D_out      // 32-bit Unsigned Radicand (Q16.16)
);

    
    // PIPELINE REGISTERS
    
    // Stage 1: Input Latches
    logic signed [15:0] dx_r, dy_r, dz_r;
    
    // 32-bit sign-extended nets to force 32-bit multiplication context
    logic signed [31:0] dx_ext, dy_ext, dz_ext;
    assign dx_ext = 32'(dx_r);
    assign dy_ext = 32'(dy_r);
    assign dz_ext = 32'(dz_r);
    
    // Stage 2: Product Registers
    logic        [31:0] dx2_r, dy2_r, dz2_r;
    
    // Stage 3: Partial Addition
    logic        [31:0] sum_xy_r;
    logic        [31:0] dz2_pipe_r;
    
    // Stage 4: Output Accumulator
    logic        [31:0] D_reg;
    
    // Valid pipeline delay line (4 clock cycles latency)
    logic        [3:0]  valid_pipe;

    
    // CONTROL PIPELINE
   
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe <= 4'b0;
        end else begin
            valid_pipe <= {valid_pipe[2:0], valid_in};
        end
    end

   
    // DATAPATH PIPELINE
    
    always_ff @(posedge clk) begin
        // Stage 1: Latch inputs
        if (valid_in) begin
            dx_r <= dx_in;
            dy_r <= dy_in;
            dz_r <= dz_in;
        end
        
        // Stage 2: Full 32-bit signed multiplication (No 16-bit truncation)
        if (valid_pipe[0]) begin
            dx2_r <= 32'(dx_ext * dx_ext);
            dy2_r <= 32'(dy_ext * dy_ext);
            dz2_r <= 32'(dz_ext * dz_ext);
        end
        
        // Stage 3: Partial Addition
        if (valid_pipe[1]) begin
            sum_xy_r   <= dx2_r + dy2_r;
            dz2_pipe_r <= dz2_r;
        end
        
        // Stage 4: Final 3D Sum-of-Squares
        if (valid_pipe[2]) begin
            D_reg <= sum_xy_r + dz2_pipe_r;
        end
    end

    
    // OUTPUT ASSIGNMENTS
    
    assign valid_out = valid_pipe[3];
    assign D_out     = D_reg;

endmodule