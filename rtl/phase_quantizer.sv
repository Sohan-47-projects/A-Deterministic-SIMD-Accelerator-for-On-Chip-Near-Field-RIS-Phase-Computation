`timescale 1ns / 1ps

module phase_quantizer #(
    parameter int B = 2  // Number of quantization bits
)(
    input  logic         clk,
    input  logic         rst_n,
    
    // Control 
    input  logic         valid_in,
    
    // Datapath inputs
    input  logic [15:0]  R_in,          // 16-bit unsigned distance (Q8.8)
    input  logic [15:0]  k_lambda,      // Precomputed BAM phase scaling constant
    
    // Outputs
    output logic         valid_out,
    output logic [B-1:0] state_out      // B-bit quantized phase state
);

    localparam logic [15:0] ROUND_BIAS = 16'(1 << (15 - B)); // To centre the states, centre 0 degrees between -45 deg and +45 deg (B=2)

    // 32-bit zero-extended operands
    logic [31:0] R_ext, k_lambda_ext;
    logic [31:0] mult_prod;

    assign R_ext        = {16'b0, R_in};
    assign k_lambda_ext = {16'b0, k_lambda}; // Unecessary zero padding for the multiplier but still did that because just in case. 
    assign mult_prod    = R_ext * k_lambda_ext; 

    // Pipeline registers
    logic [15:0]  theta_bam_r; // 16 bit LSB of mult_prod
    logic [B-1:0] state_reg;
    logic [1:0]   valid_pipe;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            theta_bam_r <= '0;
            state_reg   <= '0;
            valid_pipe  <= 2'b0;
        end else begin
            valid_pipe <= {valid_pipe[0], valid_in};
            
            // Stage 1: BAM phase scaling (Extract bits [15:0] = mod 2^16)
            if (valid_in) begin
                theta_bam_r <= mult_prod[15:0];
            end
            
            // Stage 2: Half-LSB rounding and sector extraction
            if (valid_pipe[0]) begin
                state_reg <= 16'(theta_bam_r + ROUND_BIAS) >> (16 - B);
            end
        end
    end

    assign valid_out = valid_pipe[1];
    assign state_out = state_reg;

endmodule


            // k_lambda already accounts for the /256 needed to convert
            // R_in (Q8.8) back to meters, so the 32-bit product's lower
            // 16 bits directly give the BAM phase register.
            // Bug : Previously this extracted mult_prod[23:8], an extra >>8
            // that divided the phase by a further 256x.)