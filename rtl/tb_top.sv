`timescale 1ns / 1ps

module tb_top;

    
    // SIGNALS & DUT INTERFACES
   
    logic        s_axi_aclk;
    logic        s_axi_aresetn;

    // Write Address Channel
    logic [7:0]  s_axi_awaddr;
    logic [2:0]  s_axi_awprot;
    logic        s_axi_awvalid;
    logic        s_axi_awready;

    // Write Data Channel
    logic [31:0] s_axi_wdata;
    logic [3:0]  s_axi_wstrb;
    logic        s_axi_wvalid;
    logic        s_axi_wready;

    // Write Response Channel
    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready;

    // Read Address Channel
    logic [7:0]  s_axi_araddr;
    logic [2:0]  s_axi_arprot;
    logic        s_axi_arvalid;
    logic        s_axi_arready;

    // Read Data Channel
    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready;

    // Interrupt Line
    logic        irq_out;

   
    // AXI ADDRESS MAP CONSTANTS
    
    localparam logic [7:0] ADDR_CSR          = 8'h00;
    localparam logic [7:0] ADDR_TARGET_X     = 8'h04;
    localparam logic [7:0] ADDR_TARGET_Y     = 8'h08;
    localparam logic [7:0] ADDR_TARGET_Z     = 8'h0C;
    localparam logic [7:0] ADDR_LAMBDA_SCALE = 8'h10;
    localparam logic [7:0] ADDR_PHASE_BASE   = 8'h20;

    
    // DUT INSTANTIATION
  
    axi4_lite_slave #(
        .ADDR_WIDTH (8),
        .DATA_WIDTH (32)
    ) dut (
        .s_axi_aclk    (s_axi_aclk),
        .s_axi_aresetn (s_axi_aresetn),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_awprot  (s_axi_awprot),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_arprot  (s_axi_arprot),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .irq_out       (irq_out)
    );

    
    // CLOCK GENERATION (100 MHz -> 10 ns period)

    initial begin
        s_axi_aclk = 0;
        forever #5 s_axi_aclk = ~s_axi_aclk;
    end

    
    // AXI4-LITE BUS MASTER TASKS
    
    task automatic axi_write(input logic [7:0] addr, input logic [31:0] data);
        begin
            @(posedge s_axi_aclk);
            s_axi_awaddr  <= addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wstrb   <= 4'hF;
            s_axi_wvalid  <= 1'b1;

            fork
                begin
                    do @(posedge s_axi_aclk);
                    while (!s_axi_awready);
                    s_axi_awvalid <= 1'b0;
                end
                begin
                    do @(posedge s_axi_aclk);
                    while (!s_axi_wready);
                    s_axi_wvalid <= 1'b0;
                end
            join

            s_axi_bready <= 1'b1;
            do @(posedge s_axi_aclk);
            while (!s_axi_bvalid);
            s_axi_bready <= 1'b0;
        end
    endtask

    task automatic axi_read(input logic [7:0] addr, output logic [31:0] data);
        begin
            @(posedge s_axi_aclk);
            s_axi_araddr  <= addr;
            s_axi_arvalid <= 1'b1;

            do @(posedge s_axi_aclk);
            while (!s_axi_arready);
            s_axi_arvalid <= 1'b0;

            s_axi_rready <= 1'b1;
            do @(posedge s_axi_aclk);
            while (!s_axi_rvalid);
            data = s_axi_rdata;
            s_axi_rready <= 1'b0;
        end
    endtask

    
    // AUTOMATED PROTOCOL & TIMING MONITORS
    
    int latency_counter;
    logic latency_measuring;
    int frame_counter;
    logic measuring_frame;
    int monitor_errors;
    int test_errors;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            latency_counter   <= 0;
            latency_measuring <= 0;
            frame_counter     <= 0;
            measuring_frame   <= 0;
            monitor_errors    <= 0;
        end else begin
            // 1. One-Shot Pipeline Latency Checker
            if (dut.u_ris_core.start_pulse) begin
                latency_measuring <= 1'b1;
                latency_counter   <= 0;
            end else if (latency_measuring) begin
                if (dut.u_ris_core.lane_valid_in || latency_counter > 0) begin
                    latency_counter <= latency_counter + 1;
                    if (dut.u_ris_core.lane_valid_out[0]) begin
                        latency_measuring <= 1'b0;
                        if (latency_counter != 22) begin
                            $display("[ASSERTION FAILED] Pipeline latency mismatch! Expected 22 cycles, got %0d.", latency_counter);
                            monitor_errors <= monitor_errors + 1;
                        end else begin
                            $display("  [ASSERTION PASSED] Math Lane Pipeline Latency = 22 clock cycles.");
                        end
                    end
                end
            end

            // 2. Total Frame Compute Time & Busy Flag Checker
            if (dut.u_ris_core.start_pulse) begin
                measuring_frame <= 1'b1;
                frame_counter   <= 1;
            end else if (measuring_frame) begin
                if (dut.u_ris_core.done_pulse) begin
                    measuring_frame <= 1'b0;
                    if (frame_counter != 86 && frame_counter != 87) begin
                        $display("[ASSERTION FAILED] Frame compute time mismatch! Expected 86/87 cycles, got %0d.", frame_counter);
                        monitor_errors <= monitor_errors + 1;
                    end else begin
                        $display("  [ASSERTION PASSED] Full Frame Compute Time = %0d clock cycles.", frame_counter);
                    end
                end else begin
                    frame_counter <= frame_counter + 1;
                    if (!dut.u_ris_core.busy_out) begin
                        $display("[ASSERTION FAILED] busy_out dropped unexpectedly at frame cycle %0d!", frame_counter);
                        monitor_errors <= monitor_errors + 1;
                    end
                end
            end
        end
    end

    
    // MAIN VERIFICATION TEST FLOW
    
    int file_stim, file_exp;
    int scan_ret;
    int test_case_num;
    int poll_count;
    int total_errors;

    logic [63:0] raw_stim_line;
    logic [15:0] stim_tx, stim_ty, stim_tz, stim_klambda;
    logic [31:0] exp_word;
    logic [31:0] read_val;
    logic [31:0] csr_val;

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top);

        s_axi_aresetn = 0;
        s_axi_awaddr  = '0;
        s_axi_awprot  = '0;
        s_axi_awvalid = 0;
        s_axi_wdata   = '0;
        s_axi_wstrb   = '0;
        s_axi_wvalid  = 0;
        s_axi_bready  = 0;
        s_axi_araddr  = '0;
        s_axi_arprot  = '0;
        s_axi_arvalid = 0;
        s_axi_rready  = 0;
        
        test_errors   = 0;
        test_case_num = 0;

        $display("====================================================================");
        $display("  ICARUS VERILOG: RIS PHASE ACCELERATOR HARDWARE SIMULATION         ");
        $display("====================================================================");

        file_stim = $fopen("stimulus_in.hex", "r");
        file_exp  = $fopen("expected_out.hex", "r");

        if (file_stim == 0 || file_exp == 0) begin
            $display("[FATAL ERROR] Could not open stimulus_in.hex or expected_out.hex!");
            $finish;
        end

        // Reset Sequence
        repeat (3) @(posedge s_axi_aclk);
        s_axi_aresetn = 1;
        repeat (5) @(posedge s_axi_aclk);

        while (!$feof(file_stim)) begin
            // Parse full 64-bit vector per line (immune to Icarus underscore literal grouping)
            scan_ret = $fscanf(file_stim, "%h\n", raw_stim_line);
            if (scan_ret == 1) begin
                test_case_num = test_case_num + 1;
                stim_tx      = raw_stim_line[63:48];
                stim_ty      = raw_stim_line[47:32];
                stim_tz      = raw_stim_line[31:16];
                stim_klambda = raw_stim_line[15:0];

                $display("\n--------------------------------------------------------------------");
                $display("  TEST CASE %0d: Stimulus TX=0x%04h, TY=0x%04h, TZ=0x%04h, K_LAMBDA=0x%04h",
                         test_case_num, stim_tx, stim_ty, stim_tz, stim_klambda);
                $display("--------------------------------------------------------------------");

                axi_write(ADDR_TARGET_X,     {16'h0000, stim_tx});
                axi_write(ADDR_TARGET_Y,     {16'h0000, stim_ty});
                axi_write(ADDR_TARGET_Z,     {16'h0000, stim_tz});
                axi_write(ADDR_LAMBDA_SCALE, {16'h0000, stim_klambda});

                // Trigger computation
                axi_write(ADDR_CSR, 32'h0000_0001);

                // Poll for Done flag
                poll_count = 0;
                csr_val    = '0;
                while ((csr_val[2] == 1'b0) && (poll_count < 300)) begin
                    axi_read(ADDR_CSR, csr_val);
                    poll_count = poll_count + 1;
                end

                if (csr_val[2] == 1'b0) begin
                    $display("  [FATAL] Timeout waiting for core to finish!");
                    test_errors = test_errors + 1;
                    $finish;
                end

                // Read and verify 16 output registers
                for (int w = 0; w < 16; w = w + 1) begin
                    scan_ret = $fscanf(file_exp, "%h\n", exp_word);
                    if (scan_ret != 1) begin
                        $display("  [FATAL] Missing expected output line for Test %0d, Word %02d!", test_case_num, w);
                        test_errors = test_errors + 1;
                    end

                    axi_read(ADDR_PHASE_BASE + (w * 4), read_val);

                    if (read_val !== exp_word) begin
                        $display("  [MISMATCH] Reg 0x%02h (Word %02d): Expected 0x%08h | Received 0x%08h",
                                 ADDR_PHASE_BASE + (w * 4), w, exp_word, read_val);
                        test_errors = test_errors + 1;
                    end else begin
                        $display("  [MATCH]    Reg 0x%02h (Word %02d): 0x%08h [PASS]",
                                 ADDR_PHASE_BASE + (w * 4), w, read_val);
                    end
                end

                // W1C Done flag
                axi_write(ADDR_CSR, 32'h0000_0004);
            end
        end

        $fclose(file_stim);
        $fclose(file_exp);

        @(posedge s_axi_aclk);
        total_errors = test_errors + monitor_errors;

        $display("\n====================================================================");
        if (test_case_num == 0) begin
            $display("  [FATAL ERROR] 0 test cases executed! Stimulus parsing failed.");
        end else if (total_errors == 0) begin
            $display("  VERIFICATION SUCCESS: ALL %0d TESTS & TIMING CHECKS PASSED", test_case_num);
        end else begin
            $display("  VERIFICATION FAILED: %0d TOTAL ERRORS (%0d Test, %0d Protocol/Timing)",
                     total_errors, test_errors, monitor_errors);
        end
        $display("====================================================================\n");

        $finish;
    end

endmodule