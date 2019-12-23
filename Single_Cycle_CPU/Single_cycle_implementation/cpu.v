`timescale 1ns / 1ps

//This acts like the control unit and sends the required signals to the remaining modules. 
//It detects the type of instruction from the opcode and selects the required output using a multiplexer from the outputs of the following 5 modules. 
//Since JAL, JALR, LUI and AUIPC are instructions with unique opcodes, the CPU generates the required output within itself without depending on an external module.

module CPU (
    input clk,
    input reset,
	output reg [31:0] iaddr,  //Stores current Program counter value
	output reg [31:0]  pc,     //Stores the value that is to be assigned in the next clk cycle to Program counter
    output [31:0] x31
);
	 reg [3:0] we;     // write enable signal for each byte of 32-b word
	 reg wer;
	 wire [31:0] regdata_R, regdata_I;
	 wire [31:0] regdata_L, iaddr_val;
     wire [3:0] we_S;
     wire [31:0] idata;   // data from instruction memory
     reg [31:0] daddr;  // address to data memory
     wire [31:0] drdata;  // data read from data memory reg [31:0] dwdata, // data to be written to data memory reg [4:0] rd,rs1,rs2, reg signed [31:0] imm,
	 wire [31:0] rv1, rv2;
	 reg [31:0] regdata;
	 reg [31:0] dwdata; // data to be written to data memory
	 reg [4:0] rd,rs1,rs2;
	 reg signed [31:0] imm;

    always@(posedge clk)
    begin
        if(reset)       //Other functions for reset ????
            iaddr = 0;
        else
            iaddr = pc;
    end 

    //Instantiating Imem, dmem and reg file
    imem im2(.iaddr(iaddr), .idata(idata));
    dmem d1(clk,daddr,dwdata,we,drdata);
    regfile reg1(clk,rs1,rs2,rd,regdata,wer,rv1,rv2,x31);
	 
    always@(*)     
    begin
        case(idata[6:0])
            7'b0110011:      //R type instructions
            begin
                rd = idata[11:7];
                rs1 = idata[19:15];
                rs2 = idata[24:20];
                wer = 1;
                we = 4'b0;
                regdata = regdata_R;
					 pc = iaddr+4;
            end
            7'b0010011:     //I type instructions
            begin
                rd = idata[11:7];
                rs1 = idata[19:15]; 
                imm = {{20{idata[31]}},idata[31:20]};
                wer=1;
                we=4'b0;
                regdata = regdata_I;
					 pc = iaddr+4;
            end
            7'b0000011:     //L type instructions
            begin
                rd = idata[11:7];
                rs1 = idata[19:15]; 
                imm = {{20{idata[31]}},idata[31:20]};
                wer=1;
                we=4'b0;
                daddr = rv1+imm;    
                regdata = regdata_L;
					 pc = iaddr+4;
            end
            7'b0100011:     //S type instructions
            begin
                rs1 = idata[19:15];
                rs2 = idata[24:20];
                imm = {{20{idata[31]}},idata[31:25],idata[11:7]};
                wer=0;
                daddr = rv1+imm;
					 case(idata[14:12])
					 3'b000: dwdata = {rv2[7:0],rv2[7:0],rv2[7:0],rv2[7:0]};
					 3'b001: dwdata = {rv2[15:0],rv2[15:0]};
					 3'b010: dwdata = rv2;
					 endcase
                we = we_S;
					 pc = iaddr+4;
            end
				7'b1100011:		//B type instructions
				begin
					rs1 = idata[19:15];
					rs2 = idata[24:20];
					imm = {{20{idata[31]}},idata[31],idata[7],idata[30:25],idata[11:8],1'b0};
					wer=0;
					we=4'b0;
					pc = iaddr_val;
				end
				7'b1100111:		//JALR instruction
				begin
					rs1 = idata[19:15];
					rd = idata[11:7];
					imm = {{20{idata[31]}},idata[31:20]};
					wer = 1;
					we = 4'b0;
					regdata = iaddr+4;
					pc = (rv1+imm)&32'hfffffffe;
				end
				7'b1101111:		//JAL instruction
				begin
					rd = idata[11:7];
					imm = {{11{idata[31]}},idata[31],idata[19:12],idata[20],idata[30:21],1'b0};
					pc = (iaddr+imm);
					wer = 1;
					we = 4'b0;
					regdata = iaddr+4;
				end
				7'b0010111:		//AUIPC
				begin
					rd = idata[11:7];
					imm = {idata[31:12],12'b0};
					wer = 1;
					we = 4'b0;
					regdata = iaddr+imm;
					pc = iaddr+4;
				end
				7'b0110111:		//LUI
				begin
					rd = idata[11:7];
					imm = {idata[31:12],12'b0};
					wer=1;
					we=4'b0;
					regdata = imm;
					pc = iaddr+4;
				end
			endcase
    end

    //Instantiating modules from the computational block
    R_type r1(idata,rv1,rv2,regdata_R);
    I_type i1(idata,rv1,imm,regdata_I);
    L_type l1(idata, daddr, drdata, regdata_L);
    S_type s1(idata,daddr,we_S);
	B_type b1(idata, iaddr, imm, rv1, rv2, iaddr_val);
	
endmodule
