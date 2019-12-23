`timescale 1ns / 1ps

module cpu_tb();
    reg clk, reset;     //check.. drdata and idata should be reg??????
    wire [31:0] iaddr;
    wire [31:0] idata, drdata;
    wire [31:0] dwdata;
	 wire [31:0]daddr; 
	 //wire [3:0] we;
	 //wire wer;
    //wire [31:0] x31, pc;
	 wire [4:0] rs1,rs2,rd;
	 wire [31:0] rv1,rv2;
	 wire [31:0] regdata;
	 //wire [4:0] rd,rs1,rs2;
	 //wire [31:0] imm;
	 //wire [31:0] regdata_R, regdata_I;
	 wire [31:0] pc;
	 
    CPU dut(
        .clk(clk),
        .reset(reset),
        .iaddr(iaddr),
        .idata(idata),
        .daddr(daddr),
        .drdata(drdata),
        .dwdata(dwdata),
        //.we(we),
		  .rd(rd),.rs1(rs1),.rs2(rs2),
		  //.imm(imm), 
		  .rv1(rv1), .rv2(rv2), 
		  .regdata(regdata),
		  //.offset(offset),
			//.wer(wer),
		  //.regdata_R(regdata_R), .regdata_I(regdata_I),
		  .pc(pc)
        //.x31(x31),
        //.pc(pc)
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
