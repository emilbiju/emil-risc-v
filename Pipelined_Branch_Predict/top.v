`timescale 1ns / 1ps

//Implementation Testbench for A0

module top(
    input clk
    ); //Only input from the outside is clock
	 
	 //wire reset,rdy;
	wire reset;
	wire [31:0] iaddr, pc;
	wire [31:0] x31;
	wire man_clk;
	wire [31:0] L_regdata_out, EM_daddr_out, MW_drdata_out;
	 //Input-output ports controlled by VIO and ILA
	 
	wire [35:0] ILA_CONTROL, VIO_CONTROL;
	 //Control wires used by ICON to control VIO and ILA
	 
	 
pipeline_CPU instanceA (man_clk, reset, iaddr, x31, L_regdata_out, EM_daddr_out, MW_drdata_out);

//Calling the multiplier instance


//Calls for ICON, VIO and ILA blocks
icon0 instanceB (
    .CONTROL0(ILA_CONTROL), // INOUT BUS [35:0]
    .CONTROL1(VIO_CONTROL) // INOUT BUS [35:0]
);

vio0 instanceC (
    .CONTROL(VIO_CONTROL), // INOUT BUS [35:0]
    //.CLK(clka), // IN
    .ASYNC_OUT({man_clk, reset}), // OUT BUS [1:0]
    .ASYNC_IN({iaddr, x31, L_regdata_out, EM_daddr_out, MW_drdata_out}) // IN BUS [159:0]
);

ila0 instanceD (
    .CONTROL(ILA_CONTROL), // INOUT BUS [35:0]
    .CLK(clka), // IN
    .TRIG0(iaddr), // IN BUS [31:0]
	 .TRIG1(x31) // add other o/ps
    //.TRIG2(out) // IN BUS [15:0]
);

endmodule

/*
UCF statement to be added in constraints file-
NET "clk" LOC = "C9"  | IOSTANDARD = LVCMOS33 ;
*/
