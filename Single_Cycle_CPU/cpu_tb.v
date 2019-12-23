`timescale 1ns / 1ps

module cpu_tb();
    reg clk, reset;     //check.. drdata and idata should be reg??????
    wire [31:0] iaddr;
    wire [31:0] x31, pc;

	 
    CPU dut(
        .clk(clk),
        .reset(reset),
        .iaddr(iaddr),
		  .pc(pc),
        .x31(x31)
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
