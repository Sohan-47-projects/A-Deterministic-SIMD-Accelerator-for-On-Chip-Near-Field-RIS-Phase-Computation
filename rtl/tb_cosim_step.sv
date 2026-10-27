`timescale 1ns / 1ps

module tb_cosim_step;

    logic        clk;
    logic        rst_n;
    logic [7:0]  awaddr, araddr;
    logic [2:0]  awprot, arprot;
    logic        awvalid, awready, arvalid, arready;
    logic [31:0] wdata, rdata;
    logic [3:0]  wstrb;
    logic        wvalid, wready, rvalid, rready;
    logic [1:0]  bresp, rresp;
    logic        bvalid, bready;
    logic        irq_out;

    // Instantiate DUT
    axi4_lite_slave #(
        .ADDR_WIDTH (8),
        .DATA_WIDTH (32)
    ) dut (
        .s_axi_aclk    (clk),
        .s_axi_aresetn (rst_n),
        .s_axi_awaddr  (awaddr),
        .s_axi_awprot  (awprot),
        .s_axi_awvalid (awvalid),
        .s_axi_wdata   (wdata),
        .s_axi_wstrb   (wstrb),
        .s_axi_wvalid  (wvalid),
        .s_axi_bready  (bready),
        .s_axi_araddr  (araddr),
        .s_axi_arprot  (arprot),
        .s_axi_arvalid (arvalid),
        .s_axi_rready  (rready),
        .s_axi_awready (awready),
        .s_axi_wready  (wready),
        .s_axi_bresp   (bresp),
        .s_axi_bvalid  (bvalid),
        .s_axi_arready (arready),
        .s_axi_rdata   (rdata),
        .s_axi_rresp   (rresp),
        .s_axi_rvalid  (rvalid),
        .irq_out       (irq_out)
    );

    // 100 MHz clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // AXI Write Task
    task automatic axi_write(input logic [7:0] addr, input logic [31:0] data);
        begin
            @(posedge clk);
            awaddr  <= addr;
            awvalid <= 1'b1;
            wdata   <= data;
            wstrb   <= 4'hF;
            wvalid  <= 1'b1;
            bready  <= 1'b1;

            fork
                begin
                    while (!(awready && awvalid)) @(posedge clk);
                    awvalid <= 1'b0;
                end
                begin
                    while (!(wready && wvalid)) @(posedge clk);
                    wvalid <= 1'b0;
                end
            join

            while (!bvalid) @(posedge clk);
            @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    // AXI Read Task
    task automatic axi_read(input logic [7:0] addr, output logic [31:0] data);
        begin
            @(posedge clk);
            araddr  <= addr;
            arvalid <= 1'b1;
            rready  <= 1'b1;

            while (!(arready && arvalid)) @(posedge clk);
            arvalid <= 1'b0;

            while (!rvalid) @(posedge clk);
            data = rdata;
            @(posedge clk);
            rready <= 1'b0;
        end
    endtask

    int fd_in, fd_out, scan_ret, timeout_cnt;
    logic [15:0] tx, ty, tz, k_lam;
    logic [31:0] read_val;

    initial begin
        // Reset initialization
        rst_n = 0;
        awaddr = 0; awvalid = 0; wdata = 0; wvalid = 0; bready = 0;
        araddr = 0; arvalid = 0; rready = 0;
        awprot = 0; arprot = 0; wstrb = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // Read input file
        fd_in = $fopen("hdl_input.hex", "r");
        if (fd_in == 0) begin
            $display("[FATAL] Could not open hdl_input.hex");
            $fatal(1);
        end
        scan_ret = $fscanf(fd_in, "%h %h %h %h\n", tx, ty, tz, k_lam);
        $fclose(fd_in);

        // Program Target Coordinates & BAM scale
        axi_write(8'h04, {{16{tx[15]}}, tx});
        axi_write(8'h08, {{16{ty[15]}}, ty});
        axi_write(8'h0C, {{16{tz[15]}}, tz});
        axi_write(8'h10, {16'h0000, k_lam});

        // Enable Interrupt (CSR[3]=1) and Trigger Start (CSR[0]=1)
        axi_write(8'h00, 32'h0000_0009);

        // Closed-Loop Hardware Wait: Wait for IRQ or Timeout
        timeout_cnt = 0;
        while (!irq_out && timeout_cnt < 200) begin
            @(posedge clk);
            timeout_cnt++;
        end

        if (!irq_out) begin
            $display("[FATAL] Hardware execution timeout! irq_out never asserted.");
            $fatal(1);
        end

        // Dump 16 registers
        fd_out = $fopen("hdl_output.hex", "w");
        for (int i = 0; i < 16; i++) begin
            axi_read(8'h20 + (i * 4), read_val);
            
            // Strict X-detection
            if (^read_val === 1'bX) begin
                $display("[FATAL] X-state detected on output register 0x%02h: %08h", 8'h20 + (i*4), read_val);
                $fclose(fd_out);
                $fatal(1);
            end
            
            $fwrite(fd_out, "%08h\n", read_val);
        end
        $fflush(fd_out);
        $fclose(fd_out);

        repeat (2) @(posedge clk);
        $finish;
    end

endmodule