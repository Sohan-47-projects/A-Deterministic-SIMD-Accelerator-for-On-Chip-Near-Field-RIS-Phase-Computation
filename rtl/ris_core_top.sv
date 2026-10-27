`timescale 1ns / 1ps

module ris_core_top #(
    parameter int B = 2  // Quantization bits per element (2 bits -> 4 states)
)(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control & Status (Driven by AXI4-Lite slave interface)
    input  logic        start_pulse,
    output logic        busy_out,
    output logic        done_pulse,
    
    // Target Coordinates (Q8.8 signed)
    input  logic signed [15:0] target_x,
    input  logic signed [15:0] target_y,
    input  logic signed [15:0] target_z,
    input  logic [15:0]        k_lambda,
    
    // Packed Output Register File: 512-bit flat bus (16 words x 32 bits = 256 x 2-bit states)
    output logic [511:0] phase_regs
);

    
    logic [47:0] rom_lane0 [0:63];
    logic [47:0] rom_lane1 [0:63];
    logic [47:0] rom_lane2 [0:63];
    logic [47:0] rom_lane3 [0:63];

    initial begin
        $readmemh("rom_lane0.hex", rom_lane0);
        $readmemh("rom_lane1.hex", rom_lane1);
        $readmemh("rom_lane2.hex", rom_lane2);
        $readmemh("rom_lane3.hex", rom_lane3);
    end

    
    // DISPATCH CONTROLLER
    
    logic [5:0] stride_cnt;
    logic       dispatching;
    logic       lane_valid_in;
    logic       in_flight;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stride_cnt    <= '0;
            dispatching   <= 1'b0;
            lane_valid_in <= 1'b0;
        end else begin
            if (start_pulse && !dispatching && !in_flight) begin
                dispatching   <= 1'b1;
                stride_cnt    <= '0;
                lane_valid_in <= 1'b1;
            end else if (dispatching) begin
                if (stride_cnt == 6'd63) begin
                    dispatching   <= 1'b0;
                    lane_valid_in <= 1'b0;
                    stride_cnt    <= '0;
                end else begin
                    stride_cnt    <= stride_cnt + 1'b1;
                    lane_valid_in <= 1'b1;
                end
            end else begin
                lane_valid_in <= 1'b0;
            end
        end
    end

    
    //  4-WAY SIMD VECTORIZED MATH LANES
    
    logic         lane_valid_out [0:3];
    logic [7:0]   lane_elem_idx  [0:3];
    logic [B-1:0] lane_state_out [0:3];
    
    logic signed [15:0] rom_x [0:3], rom_y [0:3], rom_z [0:3];
    
    assign {rom_x[0], rom_y[0], rom_z[0]} = rom_lane0[stride_cnt];
    assign {rom_x[1], rom_y[1], rom_z[1]} = rom_lane1[stride_cnt];
    assign {rom_x[2], rom_y[2], rom_z[2]} = rom_lane2[stride_cnt];
    assign {rom_x[3], rom_y[3], rom_z[3]} = rom_lane3[stride_cnt];

    genvar lane;
    generate
        for (lane = 0; lane < 4; lane = lane + 1) begin : gen_simd_lanes
            logic [7:0] cur_elem_idx;
            assign cur_elem_idx = {stride_cnt, 2'(lane)};
            
            lane_core #(
                .B (B)
            ) u_lane (
                .clk          (clk),
                .rst_n        (rst_n),
                .valid_in     (lane_valid_in),
                .elem_idx_in  (cur_elem_idx),
                .target_x     (target_x),
                .target_y     (target_y),
                .target_z     (target_z),
                .elem_x       (rom_x[lane]),
                .elem_y       (rom_y[lane]),
                .elem_z       (rom_z[lane]),
                .k_lambda     (k_lambda),
                .valid_out    (lane_valid_out[lane]),
                .elem_idx_out (lane_elem_idx[lane]),
                .state_out    (lane_state_out[lane])
            );
        end
    endgenerate

    
    // OUTPUT PACKER & FRAME DRAIN CONTROLLER

    logic [511:0] reg_file_flat;
    logic [5:0]   drain_cnt;
    logic         done_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_file_flat <= '0;
            drain_cnt     <= '0;
            in_flight     <= 1'b0;
            done_reg      <= 1'b0;
        end else begin
            done_reg <= 1'b0;
            
            if (start_pulse && !dispatching && !in_flight) begin
                in_flight <= 1'b1;
            end

            // When valid output stream emerges from SIMD lanes
            if (lane_valid_out[0]) begin
                // Pack 4 x 2-bit states (8 bits total) into the flat 512-bit register
                reg_file_flat[drain_cnt * 8 +: 8] <= {
                    lane_state_out[3],
                    lane_state_out[2],
                    lane_state_out[1],
                    lane_state_out[0]
                };
                
                if (drain_cnt == 6'd63) begin
                    drain_cnt <= '0;
                    in_flight <= 1'b0;
                    done_reg  <= 1'b1;
                end else begin
                    drain_cnt <= drain_cnt + 1'b1;
                end
            end
        end
    end

    
    // OUTPUT ASSIGNMENTS
  
    assign busy_out   = dispatching || in_flight;
    assign done_pulse = done_reg;
    assign phase_regs = reg_file_flat;

endmodule