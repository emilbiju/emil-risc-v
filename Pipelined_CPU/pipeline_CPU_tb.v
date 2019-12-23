`timescale 1ns / 1ps

module pipeline_cpu_tb();
    reg clk, reset;   
    wire [31:0] iaddr;
    wire [31:0] x31;
	 wire [31:0] EM_daddr_out;
	 wire [31:0] MW_drdata_out;
	 wire [31:0] L_regdata_out;
	 
    pipeline_CPU dut(
        .clk(clk),
        .reset(reset),
        .iaddr(iaddr),
        .x31(x31),
		  .EM_daddr_out(EM_daddr_out),
		  .MW_drdata_out(MW_drdata_out),
		  .L_regdata_out(L_regdata_out)
    );
	 
    always #5 clk = ~clk;
    initial begin
        clk = 0;
        reset = 1;
        #30;
        reset = 0;
		  $finish;
    end
endmodule
