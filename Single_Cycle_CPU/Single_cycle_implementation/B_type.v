`timescale 1ns / 1ps
//Module for branch instructions - The new pc value is generated by this module
module B_type(
    input [31:0] instr,
	 input [31:0] iaddr,
	 input signed [31:0] imm,
    input signed [31:0] in1, in2,
    output reg [31:0] out
);
	 wire [31:0] tmp1, tmp2;
	 assign tmp1 = in1;
	 assign tmp2 = in2;
    always@(instr or in1 or in2 or iaddr or imm)
    begin
		case(instr[14:12])
        3'b000: out = (in1==in2)? (iaddr+imm) : (iaddr+4);
		  3'b001: out = (in1!=in2)? (iaddr+imm) : (iaddr+4);
        3'b100: out = (in1<in2)? (iaddr+imm) : (iaddr+4);
        3'b101: out = (in1>=in2)? (iaddr+imm) : (iaddr+4);
        3'b110: out = (tmp1<tmp2)? (iaddr+imm) : (iaddr+4);
        3'b111: out = (tmp1>=tmp2)? (iaddr+imm) : (iaddr+4);
		endcase
	 end
endmodule
