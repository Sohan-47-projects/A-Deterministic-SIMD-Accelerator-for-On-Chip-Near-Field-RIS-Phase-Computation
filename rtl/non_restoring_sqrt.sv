`timescale 1ns / 1ps

module non_restoring_sqrt (
    input  logic        clk,
    input  logic        rst_n,
    
    // Input Stream
    input  logic        valid_in,
    input  logic [31:0] D_in,      // 32-bit unsigned radicand (Q16.16)
    
    // Output Stream
    output logic        valid_out,
    output logic [15:0] Q_out      // 16-bit unsigned root (Q8.8)
);

    
    // PIPELINE REGISTERS: 16 Stages (0 to 15)
    // Residuals (R) stored as 18-bit signed values.
   
    logic signed [17:0] R_stage [0:15];
    logic        [15:0] Q_stage [0:15];
    logic        [31:0] D_stage [0:15];
    logic        [15:0] valid_pipe;

    
    // STAGE 0: Initial 2-bit trial subtraction (Always D[31:30] - 1)
   
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe[0] <= 1'b0;
            R_stage[0]    <= '0;
            Q_stage[0]    <= '0;
            D_stage[0]    <= '0;
        end else begin
            valid_pipe[0] <= valid_in;
            if (valid_in) begin
                D_stage[0] <= D_in << 2;
                
                // Trial subtraction: d0 - 1
                R_stage[0] <= $signed({16'b0, D_in[31:30]}) - 18'sd1;
                Q_stage[0] <= (D_in[31:30] >= 2'b01) ? 16'd1 : 16'd0;
            end
        end
    end

    
    // STAGES 1 to 15: Pipelined Recurrence Stages
    
    genvar i;
    generate
        for (i = 1; i < 16; i = i + 1) begin : gen_sqrt_stages
            
            // 20-bit intermediate signed nets to prevent sign-drop or overflow
            logic signed [19:0] next_R_shifted;
            logic signed [19:0] trial_term;
            logic signed [19:0] test_R_wide;
            
            // Arithmetic left-shift preserves sign bits, then add next 2 radicand bits
            assign next_R_shifted = $signed({R_stage[i-1], D_stage[i-1][31:30]});
            
            // If prev R >= 0: Subtract (4Q + 1) -> {2'b00, Q, 2'b01}
            // If prev R <  0: Add      (4Q + 3) -> {2'b00, Q, 2'b11}
            assign trial_term   = (R_stage[i-1] >= 0) ? 
                                  $signed({2'b00, Q_stage[i-1], 2'b01}) : 
                                  $signed({2'b00, Q_stage[i-1], 2'b11});

            assign test_R_wide  = (R_stage[i-1] >= 0) ? 
                                  (next_R_shifted - trial_term) : 
                                  (next_R_shifted + trial_term);

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    valid_pipe[i] <= 1'b0;
                    R_stage[i]    <= '0;
                    Q_stage[i]    <= '0;
                    D_stage[i]    <= '0;
                end else begin
                    valid_pipe[i] <= valid_pipe[i-1];
                    if (valid_pipe[i-1]) begin
                        D_stage[i] <= D_stage[i-1] << 2;
                        R_stage[i] <= test_R_wide[17:0];
                        
                        // If test residual is non-negative, bit fits (1), else 0
                        if (test_R_wide >= 0) begin
                            Q_stage[i] <= {Q_stage[i-1][14:0], 1'b1};
                        end else begin
                            Q_stage[i] <= {Q_stage[i-1][14:0], 1'b0};
                        end
                    end
                end
            end
        end
    endgenerate

    
    // OUTPUTS
  
    assign valid_out = valid_pipe[15];
    assign Q_out     = Q_stage[15];

endmodule