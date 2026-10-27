`timescale 1ns / 1ps

module axi4_lite_slave #(
    parameter int ADDR_WIDTH = 8,
    parameter int DATA_WIDTH = 32
)(
    // Global Clock & Reset
    input  logic                      s_axi_aclk,
    input  logic                      s_axi_aresetn,
    
    // AXI4-Lite Write Address Channel
    input  logic [ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  logic [2:0]                s_axi_awprot,
    input  logic                      s_axi_awvalid,
    output logic                      s_axi_awready,
    
    // AXI4-Lite Write Data Channel
    input  logic [DATA_WIDTH-1:0]     s_axi_wdata,
    input  logic [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  logic                      s_axi_wvalid,
    output logic                      s_axi_wready,
    
    // AXI4-Lite Write Response Channel
    output logic [1:0]                s_axi_bresp,
    output logic                      s_axi_bvalid,
    input  logic                      s_axi_bready,
    
    // AXI4-Lite Read Address Channel
    input  logic [ADDR_WIDTH-1:0]     s_axi_araddr,
    input  logic [2:0]                s_axi_arprot,
    input  logic                      s_axi_arvalid,
    output logic                      s_axi_arready,
    
    // AXI4-Lite Read Data Channel
    output logic [DATA_WIDTH-1:0]     s_axi_rdata,
    output logic [1:0]                s_axi_rresp,
    output logic                      s_axi_rvalid,
    input  logic                      s_axi_rready,
    
    // Interrupt Output
    output logic                      irq_out
);

    localparam logic [ADDR_WIDTH-1:0] ADDR_CSR          = 8'h00;
    localparam logic [ADDR_WIDTH-1:0] ADDR_TARGET_X     = 8'h04;
    localparam logic [ADDR_WIDTH-1:0] ADDR_TARGET_Y     = 8'h08;
    localparam logic [ADDR_WIDTH-1:0] ADDR_TARGET_Z     = 8'h0C;
    localparam logic [ADDR_WIDTH-1:0] ADDR_LAMBDA_SCALE = 8'h10;
    localparam logic [ADDR_WIDTH-1:0] ADDR_PHASE_BASE   = 8'h20;

   
    logic signed [15:0] reg_target_x;
    logic signed [15:0] reg_target_y;
    logic signed [15:0] reg_target_z;
    logic [15:0]        reg_k_lambda;
    logic               reg_int_en;
    logic               reg_done_sticky;
    logic               start_strobe;

    // Core Top Interconnect Signals
    logic         core_busy;
    logic         core_done;
    logic [511:0] core_phase_regs;

    
    // AXI4-LITE WRITE TRANSACTION LOGIC
    
    logic [ADDR_WIDTH-1:0] axi_awaddr;
    logic                  aw_received;
    logic                  w_received;
    logic                  write_en;
    logic [ADDR_WIDTH-1:0] effective_write_addr;

    assign write_en = (s_axi_awvalid || aw_received) && (s_axi_wvalid || w_received) && !s_axi_bvalid;
    assign effective_write_addr = aw_received ? axi_awaddr : s_axi_awaddr;

    always_ff @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_awready   <= 1'b0;
            s_axi_wready    <= 1'b0;
            s_axi_bvalid    <= 1'b0;
            s_axi_bresp     <= 2'b00;
            aw_received     <= 1'b0;
            w_received      <= 1'b0;
            axi_awaddr      <= '0;
            
            reg_target_x    <= '0;
            reg_target_y    <= '0;
            reg_target_z    <= '0;
            reg_k_lambda    <= 16'h6400;
            reg_int_en      <= 1'b0;
            reg_done_sticky <= 1'b0;
            start_strobe    <= 1'b0;
        end else begin
            start_strobe <= 1'b0;

            // Address Handshake
            if (s_axi_awvalid && !aw_received && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                aw_received   <= 1'b1;
                axi_awaddr    <= s_axi_awaddr;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // Data Handshake
            if (s_axi_wvalid && !w_received && !s_axi_bvalid) begin
                s_axi_wready <= 1'b1;
                w_received   <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end

            // Atomic Register Write Execution
            if (write_en) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
                aw_received  <= 1'b0;
                w_received   <= 1'b0;

                case (effective_write_addr)
                    ADDR_CSR: begin
                        if (s_axi_wdata[0]) start_strobe    <= 1'b1;
                        if (s_axi_wdata[2]) reg_done_sticky <= 1'b0;
                        reg_int_en <= s_axi_wdata[3];
                    end
                    ADDR_TARGET_X:     reg_target_x <= s_axi_wdata[15:0];
                    ADDR_TARGET_Y:     reg_target_y <= s_axi_wdata[15:0];
                    ADDR_TARGET_Z:     reg_target_z <= s_axi_wdata[15:0];
                    ADDR_LAMBDA_SCALE: reg_k_lambda <= s_axi_wdata[15:0];
                    default: ;
                endcase
            end

            if (core_done) begin
                reg_done_sticky <= 1'b1;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    
    // AXI4-LITE READ TRANSACTION LOGIC
    
    logic [3:0] phase_word_idx;
    assign phase_word_idx = 4'((s_axi_araddr - ADDR_PHASE_BASE) >> 2);

    always_ff @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (~s_axi_arready && s_axi_arvalid && ~s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                s_axi_rresp   <= 2'b00;

                // Memory-Mapped Read Decoder
                if (s_axi_araddr >= ADDR_PHASE_BASE && s_axi_araddr < (ADDR_PHASE_BASE + 8'd64)) begin
                    // Slice 32-bit word directly from the flat 512-bit vector
                    s_axi_rdata <= core_phase_regs[phase_word_idx * 32 +: 32];
                end else begin
                    case (s_axi_araddr)
                        ADDR_CSR:          s_axi_rdata <= {28'b0, reg_int_en, reg_done_sticky, core_busy, 1'b0};
                        ADDR_TARGET_X:     s_axi_rdata <= {{16{reg_target_x[15]}}, reg_target_x};
                        ADDR_TARGET_Y:     s_axi_rdata <= {{16{reg_target_y[15]}}, reg_target_y};
                        ADDR_TARGET_Z:     s_axi_rdata <= {{16{reg_target_z[15]}}, reg_target_z};
                        ADDR_LAMBDA_SCALE: s_axi_rdata <= {16'b0, reg_k_lambda};
                        default:           s_axi_rdata <= 32'hDEAD_BEEF;
                    endcase
                end
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // Interrupt Output
    assign irq_out = reg_done_sticky & reg_int_en;

   
    // RIS CORE TOP INSTANTIATION
    
    ris_core_top #(
        .B (2)
    ) u_ris_core (
        .clk         (s_axi_aclk),
        .rst_n       (s_axi_aresetn),
        .start_pulse (start_strobe),
        .busy_out    (core_busy),
        .done_pulse  (core_done),
        .target_x    (reg_target_x),
        .target_y    (reg_target_y),
        .target_z    (reg_target_z),
        .k_lambda    (reg_k_lambda),
        .phase_regs  (core_phase_regs)
    );

endmodule