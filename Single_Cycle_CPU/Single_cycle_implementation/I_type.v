`timescale 1ns / 1ps

//Module for immediate instructions
module I_type(
    input [31:0] instr,
    input signed [31:0] in1,
    input signed [31:0] imm,
    output reg [31:0] out
);
    wire [31:0] tmp1;
    wire [11:0] tmp2;
    assign tmp1 = in1;
    assign tmp2 = imm;
    always @(instr or in1 or imm) 
    begin
    	case(instr[14:12])
        3'b000: out = in1+imm;          //addi
        3'b010: out = in1<imm;          //slti
        3'b011: out= tmp1<tmp2;         //sltiu
        3'b100: out = in1 ^ imm;        //xori
        3'b110: out = in1 | imm;        //ori
        3'b111: out = in1 & imm;        //andi
        3'b001: out = in1<<imm[4:0];    //slli
        3'b101:
        begin
            if(instr[30])           //srli
               out = in1>>imm[4:0]; 
            else                    //srai
               out = in1>>>imm[4:0];
        end
        endcase
    end
endmodule