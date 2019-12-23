`timescale 1ns / 1ps
//Regfile module
    
module regfile(
    input clk,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] regdata,
    input wer,
    output [31:0] rv1,
    output [31:0] rv2,
	 output [31:0] x31
);
    reg [31:0] r[0:31];
    integer i;
    initial begin
		r[31] = 0;
        for(i=0; i<31; i = i+1)
            r[i]=i;
    end
    assign rv1 = r[rs1];
    assign rv2 = r[rs2];
	 assign x31 = r[31];
    always @(posedge clk) 
    begin
        if(wer && rd!=0)
            r[rd] = regdata;
    end
endmodule
