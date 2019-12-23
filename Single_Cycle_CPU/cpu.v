`timescale 1ns / 1ps

module imem(
    input [31:0] iaddr,
    output [31:0] idata
);
    reg [31:0] i_arr[0:31];
    initial begin
		//$readmemh("imem2_ini.mem",i_arr);
		$readmemh("imem1_ini.mem",i_arr);
		//$readmemh("imem_lw.mem",i_arr);
	 end
    assign idata = i_arr[iaddr[31:2]];
endmodule

module dmem(
    input clk,
    input [31:0] daddr,
    input [31:0] dwdata,
    input [3:0] we,
    output [31:0] drdata
);
    reg [7:0] m[0:127];
	 wire [31:0] add0,add1,add2,add3;
	 initial $readmemh("dmem_ini.mem",m);
		
	 assign add0 = (daddr & 32'hfffffffc)+ 32'h00000000;
	 assign add1 = (daddr & 32'hfffffffc)+ 32'h00000001;
	 assign add2 = (daddr & 32'hfffffffc)+ 32'h00000002;
	 assign add3 = (daddr & 32'hfffffffc)+ 32'h00000003;
	 
	 assign drdata = {m[add3], m[add2], m[add1], m[add0]};
	 
	 always@( posedge clk)
	 begin
		if(we[0]==1)
			m[add0] = dwdata[7:0];
		if(we[1]==1)
			m[add1] = dwdata[15:8];
		if(we[2]==1)
			m[add2] = dwdata[23:16];
		if(we[3]==1)
			m[add3] = dwdata[31:24];
	 end
endmodule

module regfile(
    input clk,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] regdata,
    input wer,
    output [31:0] rv1,
    output [31:0] rv2
);
    reg [31:0] r[0:31];
    integer i;
    initial begin
        for(i=0; i<32; i = i+1)
            r[i]=i;
    end
    assign rv1 = r[rs1];
    assign rv2 = r[rs2];
    always @(posedge clk) 
    begin
        if(wer && rd!=0)
            r[rd] = regdata;
    end
endmodule

// Call the CPU and provide inputs
module CPU (
    input clk,
    input reset,
    output reg [31:0] iaddr,  // address to instruction memory
    output [31:0] idata,   // data from instruction memory
    output reg [31:0] daddr,  // address to data memory
    output [31:0] drdata,  // data read from data memory
    output reg [31:0] dwdata, // data to be written to data memory
	 output reg [4:0] rd,rs1,rs2,
	 output [31:0] rv1, rv2,
	 output reg [31:0] regdata,
	 output reg [31:0]  pc
);
    
	 reg [3:0] we;     // write enable signal for each byte of 32-b word
	 reg signed [31:0] imm;
	 reg wer;
	 wire [31:0] regdata_R, regdata_I;
	 wire [31:0] regdata_L, iaddr_val;
    wire [3:0] we_S;
	 wire [31:0] imem_tmp_iaddr;
	 assign imem_tmp_iaddr = iaddr<<2;

    always@(posedge clk)
    begin
        if(reset)       //Other functions for reset ????
            iaddr = 0;
        else
            iaddr = pc;
    end
 
    imem im2(.iaddr(imem_tmp_iaddr), .idata(idata));	
    dmem d1(clk,daddr,dwdata,we,drdata);
    regfile reg1(clk,rs1,rs2,rd,regdata,wer,rv1,rv2);
	 
    always@(idata or regdata_R or regdata_I or regdata_L or we_S or iaddr or rv1 or rv2 or imm or iaddr_val)     
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
					 pc = iaddr+1;
            end
            7'b0010011:     //I type instructions
            begin
                rd = idata[11:7];
                rs1 = idata[19:15]; 
                imm = {{20{idata[31]}},idata[31:20]};
                wer=1;
                we=4'b0;
                regdata = regdata_I;
					 pc = iaddr+1;
            end
            7'b0000011:     //L type instructions
            begin
                rd = idata[11:7];
                rs1 = idata[19:15]; 
                imm = {{20{idata[31]}},idata[31:20]};
                wer=1;
                we=4'b0;
                daddr = rv1+imm;    //imm set as signed????
                regdata = regdata_L;
					 pc = iaddr+1;
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
                //dwdata = rv2;
                we = we_S;
					 pc = iaddr+1;
            end
				7'b1100011:		//B type instructions
				begin
					rs1 = idata[19:15];
					rs2 = idata[24:20];
					imm = {{20{idata[31]}},idata[31],idata[7],idata[30:25],idata[11:8]};
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
					regdata = iaddr+1;
					pc = rv1+imm;
				end
				7'b1101111:		//JAL instruction
				begin
					rd = idata[11:7];
					imm = {{11{idata[31]}},idata[31],idata[19:12],idata[20],idata[30:21],1'b0};
					pc = iaddr+imm;
					wer = 1;
					we = 4'b0;
					regdata = iaddr+1;
				end
				7'b0010111:		//AUIPC
				begin
					rd = idata[11:7];
					imm = {idata[31:12],12'b0};
					wer = 1;
					we = 4'b0;
					regdata = iaddr+imm;
					pc = iaddr+1;
				end
				7'b0110111:		//LUI
				begin
					rd = idata[11:7];
					imm = {idata[31:12],12'b0};
					wer=1;
					we=4'b0;
					regdata = imm;
					pc = iaddr+1;
				end
			endcase
    end

    R_type r1(idata,rv1,rv2,regdata_R);
    I_type i1(idata,rv1,imm,regdata_I);
    L_type l1(idata, daddr, drdata, regdata_L);
    S_type s1(idata,daddr,we_S);
	 B_type b1(idata, iaddr, imm, rv1, rv2, iaddr_val);
endmodule

module R_type(
    input [31:0] instr,
    input signed [31:0] in1,
    input signed [31:0] in2,
    output reg[31:0] out
);
    wire [31:0] tmp1;
    wire [31:0] tmp2;
    assign tmp1 = in1;
    assign tmp2 = in2;

    always @(instr or in1 or in2) 
    begin
    	case({instr[30],instr[14:12]})
        4'b0000:    out = in1+in2;          //add
        4'b1000:    out = in1-in2;          //sub
        4'b0001:    out = in1<<in2[4:0];	//sll - check [4:0] correct or not??
        4'b0010:    out = in1<in2;          //slt
        4'b0011:    out = tmp1<tmp2;        //sltu
        4'b0100:    out = in1^in2;          //xor
        4'b0101:    out = in1>>in2[4:0];    //srl
        4'b1101:    out = in1>>>in2[4:0];   //sra
        4'b0110:    out = in1|in2;          //or
        4'b0111:    out = in1&in2;          //and
        endcase
    end
endmodule

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

module L_type(
    input [31:0] instr,
    input [31:0] daddr,
    input [31:0] drdata,
    output reg[31:0] out
);
    

	 reg [31:0] offset;
    always@(instr or daddr or drdata)
    begin   
        offset = (daddr[1:0]<<3);  //offset = 8*rv1[1:0]-1 ,eg, 8*1=8, so drdata[15:8]
       
        case(instr[14:12])
        3'b000: out = {{24{drdata[offset+7]}}, drdata[offset +: 8]};    //LB
        3'b001: out = {{16{drdata[offset+15]}}, drdata[offset +: 16]};   //LH
        3'b010: out = drdata;                                   //LW
        3'b100: out = {24'b0, drdata[offset +: 8]};             //LBU
        3'b101: out = {16'b0, drdata[offset +: 16]};            //LHU
        endcase
    end
endmodule

module S_type(
    input [31:0] instr,
    input [31:0] daddr,
    output reg[3:0] we
);
    always@(instr or daddr)
    begin   
        case(instr[14:12])
        3'b000: we = (4'b0001)<<daddr[1:0];     //SB
        3'b001: we = (4'b0011)<<daddr[1:0];     //SH
        3'b010: we = 4'b1111;                   //SW
        endcase
    end
endmodule

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
        3'b000: out = (in1==in2)? (iaddr+imm) : (iaddr+1);
		  3'b001: out = (in1!=in2)? (iaddr+imm) : (iaddr+1);
        3'b100: out = (in1<in2)? (iaddr+imm) : (iaddr+1);
        3'b101: out = (in1>=in2)? (iaddr+imm) : (iaddr+1);
        3'b110: out = (tmp1<tmp2)? (iaddr+imm) : (iaddr+1);
        3'b111: out = (tmp1>=tmp2)? (iaddr+imm) : (iaddr+1);
		endcase
	 end
endmodule
