`timescale 1ns / 1ps

//IMEM module
    
module imem(
    input [31:0] iaddr,
    output [31:0] idata
);
    reg [31:0] i_arr[0:31];
    initial begin
		$readmemh("imem1_ini.mem",i_arr);
		//$readmemh("imem5_ini.mem",i_arr);
	 end
    assign idata = i_arr[iaddr[31:2]];
endmodule
