`timescale 1ns / 1ps

//IMEM module
module imem(
    input [31:0] iaddr,
    output [31:0] idata
);
    reg [31:0] i_arr[0:31];
    initial begin
		$readmemh("imem2_ini.mem",i_arr);
	end
    assign idata = i_arr[iaddr[31:2]];
endmodule


//DMEM module
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

// IF/ID pipeline register
module IF_ID_reg(
    input clk,
	 input flag,
	 input pc_replace,
    input [31:0] idata_in, 
    input [31:0] iaddr_in, 
    output reg [31:0] FD_idata_out, 
    output reg [31:0] FD_iaddr_out
);
    always@(posedge clk)
    begin
		if(flag==1)
			begin
		  //If pc_replace=1 due to branch, then replace idata_out with ADDI R0,R0,0
        FD_idata_out = pc_replace?(32'b00000000000000000000000000010011):(idata_in);
        FD_iaddr_out = iaddr_in;
		  end
    end
endmodule    

// ID/EX pipeline register
module ID_EX_reg(
    input clk,
	 input flag,
	 input pc_replace,
    //Register file inputs
    input bit_th_in,
    input [4:0] rs1_in,
    input [4:0] rs2_in,
    input [4:0] rd_in,
    input [2:0] funct3_in,
    input [6:0] op,
    input wer_in,
    input [31:0] rv1_in,
    input [31:0] rv2_in,
    input [31:0] x31_in,
    //Control block inputs
    input [3:0] we_in,
    input signed [31:0] imm_in,
    input [31:0] iaddr_in,
    //Register file outputs
    output reg bit_th_out,
    output reg [4:0] DE_rs1_out,
    output reg [4:0] DE_rs2_out,
    output reg [4:0] DE_rd_out,
    output reg [2:0] DE_funct3_out,
    output reg [6:0] DE_op_out,
    output reg DE_wer_out,
    output reg [31:0] DE_rv1_out,
    output reg [31:0] DE_rv2_out,
    output reg [31:0] DE_x31_out,
    //Control block outputs
    output reg [3:0] DE_we_out,
    output reg signed [31:0] DE_imm_out,
    output reg [31:0] DE_iaddr_out,
	 output reg DE_flag_out
);

    always@(posedge clk)
    begin
		  if(flag==1)
		  begin
        DE_rs1_out = rs1_in;
        DE_rs2_out = rs2_in;
        DE_rd_out = rd_in;
        DE_funct3_out = funct3_in;
        DE_op_out = op;
        DE_imm_out = imm_in;
        DE_iaddr_out = iaddr_in;
        DE_wer_out =  wer_in&!pc_replace&flag;
        DE_we_out = we_in&{!pc_replace&flag,!pc_replace&flag,!pc_replace&flag,!pc_replace&flag};
        DE_x31_out = x31_in;
        DE_rv1_out = rv1_in;
        DE_rv2_out = rv2_in;
        bit_th_out = bit_th_in;
		  DE_flag_out = flag;
		  end
    end
endmodule

// EX/MEM pipeline register
module EX_MEM_reg(
    input clk,
	 input act,
    input [6:0] DE_op_out,
    input [2:0] DE_funct3_out,
    input [31:0] AU_daddr_out,
    input [3:0] AU_we_out,
    input DE_wer_out,
    input [4:0] DE_rd_out,
    input [31:0] AU_regdata_out,
    input [31:0] AU_dwdata_out,
	 input pc_replace,
	 input flag,
    output reg [6:0] EM_op_out,
    output reg [2:0] EM_funct3_out,
    output reg [31:0] EM_daddr_out,
    output reg [3:0] EM_we_out,
    output reg EM_wer_out,
    output reg [4:0] EM_rd_out,
    output reg [31:0] EM_regdata_out,
    output reg [31:0] EM_dwdata_out,
	 output reg EM_pc_replace_out,
	 output reg EM_flag_out 
);
    always@(posedge clk)
    begin
        EM_op_out = DE_op_out;
        EM_funct3_out = DE_funct3_out;
        EM_daddr_out = AU_daddr_out;
        EM_we_out = AU_we_out&{act,act,act,act};
        EM_wer_out = DE_wer_out&act;
        EM_rd_out = DE_rd_out;
        EM_regdata_out = AU_regdata_out;
        EM_dwdata_out = AU_dwdata_out;
		  EM_pc_replace_out = pc_replace;
		  EM_flag_out = flag;
    end
endmodule

// MEM/WB pipeline register
module MEM_WB_reg(
    input clk,
    input [6:0] EM_op_out,
    input  [2:0] EM_funct3_out,
    input  [31:0] EM_daddr_out,
    input  EM_wer_out,
    input  [4:0] EM_rd_out,
    input  [31:0] EM_regdata_out,
    input [31:0] DMEM_drdata_out,
    output reg [6:0] MW_op_out,
    output reg [2:0] MW_funct3_out,
    output reg [31:0] MW_daddr_out,
    output reg MW_wer_out,
    output reg [4:0] MW_rd_out,
    output reg [31:0] MW_regdata_out,
    output reg [31:0] MW_drdata_out
);
    always@(posedge clk)
    begin
        MW_funct3_out = EM_funct3_out;
        MW_op_out = EM_op_out;
        MW_daddr_out = EM_daddr_out;
        MW_wer_out = EM_wer_out;
        MW_rd_out = EM_rd_out;
        MW_regdata_out = EM_regdata_out;
        MW_drdata_out = DMEM_drdata_out;
    end
endmodule

// Below is a register to store the value written in the register file in the same clock cycle.
// If the 1st clock cycle during a stall uses the MW/WB forwarding, then the value will be written in the next
//clock cycle. This can only be read if there is forwarding from this WB_reg. 
module WB_reg(
	input clk,
	input [4:0] MW_rd_out,
	input [31:0] L_regdata_out,
	input MW_wer_out,
	output reg [4:0] WB_rd_out,
	output reg [31:0] WB_regdata_out,
	output reg WB_wer_out
);
	always@(posedge clk)
	begin
		WB_rd_out = MW_rd_out;
		WB_regdata_out = L_regdata_out;
		WB_wer_out = MW_wer_out;
	end
endmodule

//Register file module
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
				always @(posedge clk) 
            begin
                if(wer && rd!=0)
                    r[rd] = regdata;
            end
            assign rv1 = r[rs1];
            assign rv2 = r[rs2];
            assign x31 = r[31];
            
endmodule

// All other modules are instantiated in pipeline_CPU. Updates and gives PC value to IMEM.
module pipeline_CPU(
    input clk,
    input reset,
    output reg [31:0] iaddr,  
    output [31:0] x31,
	 output [31:0] L_regdata_out,
	 output [31:0] EM_daddr_out,
	 output [31:0] MW_drdata_out
);

	//Declaration of wires used in instantiation.
	 wire [31:0] FD_idata_out;
    wire [31:0] FD_iaddr_out;
	 reg flag;
	 reg act;
	 wire [31:0] pc_new;
	 wire pc_replace;
	 wire pc_JALR;
	 reg [31:0] iaddr_upd;
    wire [4:0] DE_rs1_out;
    wire [4:0] DE_rs2_out;
    wire [4:0] DE_rd_out;
    wire DE_wer_out;
	 wire [4:0] rs1;
	 wire [4:0] rs2;
	 wire [4:0] rd;
	 wire [31:0] CU_imm_out;
	 wire [31:0] rv1;
	 wire [31:0] rv2;
    wire [31:0] DE_rv1_out;
    wire [31:0] DE_rv2_out;
    wire [31:0] DE_x31_out;
    wire [3:0] DE_we_out;
    wire signed [31:0] DE_imm_out;
    wire [31:0] DE_iaddr_out;
	 reg [31:0] AU_rv1_in;
	 reg [31:0] AU_rv2_in;
    wire [3:0] EM_we_out;
    wire EM_wer_out;
	 wire [6:0] EM_op_out;
    wire [4:0] EM_rd_out;
    wire [31:0] EM_regdata_out;
    wire [31:0] EM_dwdata_out;
    wire MW_wer_out;
    wire [4:0] MW_rd_out;
    wire [31:0] MW_regdata_out;
	 wire [31:0] MW_daddr_out;
	 wire [31:0] DMEM_drdata_out;
	 wire [4:0] WB_rd_out;
	 wire [31:0] WB_regdata_out;
	 wire WB_wer_out;
	 
	 //PC update - here, PC is called iaddr
    always@(posedge clk)
        begin
            if(reset)       
                iaddr = 0;
				else if(flag==0 || pc_replace==0)
					iaddr = iaddr+4-iaddr_upd;
            else if(pc_JALR==0)
                iaddr = iaddr+4-iaddr_upd+pc_new; //DEFINE PC.
				else
					 iaddr = pc_new;
        end
	 
	 wire [31:0] idata;
    wire CU_wer_out;
    wire [3:0] CU_we_out,AU_we_out;
    wire [2:0] DE_funct3_out,EM_funct3_out,MW_funct3_out;
    wire [6:0] DE_op_out,MW_op_out;
    wire [31:0] AU_regdata_out,AU_pc_out,AU_daddr_out,AU_dwdata_out;
	 
	 //Assigning values to rs1,rs2,rd
	 assign rs1 = FD_idata_out[19:15];
	 assign rs2 = FD_idata_out[24:20];
	 assign rd = FD_idata_out[11:7];
	 wire [4:0] rd_final;
	 assign rd_final = (iaddr<16)?0:MW_rd_out;

	 always@(*)
	 begin
		iaddr_upd=32'b0;
		flag=1;
		act=1;
		if(EM_rd_out == DE_rs1_out && EM_wer_out == 1 && iaddr>=12)
		begin
			if(EM_op_out == 7'b0000011)
			begin
				iaddr_upd = 4;
				flag=0;
				act=0;
			end
			else
				begin
				AU_rv1_in = EM_regdata_out;
				end
		end
		else if(MW_rd_out == DE_rs1_out && MW_wer_out == 1 && iaddr>=16)
			begin
			AU_rv1_in = L_regdata_out;
			end
		else if(WB_rd_out == DE_rs1_out && WB_wer_out == 1 && iaddr>=20)
			begin
			AU_rv1_in = WB_regdata_out;
			end
		else
			begin
			AU_rv1_in = DE_rv1_out;
			end
		if(EM_rd_out == DE_rs2_out && EM_wer_out == 1 && iaddr>=12 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
		begin
			if(EM_op_out == 7'b0000011)
			begin
				iaddr_upd = 4;
				flag=0;
				act=0;
			end
			else
			begin
			AU_rv2_in = EM_regdata_out;
			end
		end
		else if(MW_rd_out == DE_rs2_out && MW_wer_out == 1 && iaddr>=16 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
			begin
			AU_rv2_in = L_regdata_out;
			end
		else if(WB_rd_out == DE_rs2_out && WB_wer_out == 1 && iaddr>=20 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
			begin
			AU_rv2_in = WB_regdata_out;
			end
		else
			begin
			AU_rv2_in = DE_rv2_out;
			end
	 end

	//Instantiation of all modules used in the architecture. This allows output of 1 module to be used by another.
    imem im1(.iaddr(iaddr), .idata(idata));
    IF_ID_reg fd1(.clk(clk),.flag(flag),.pc_replace(pc_replace),.idata_in(idata), .iaddr_in(iaddr), .FD_idata_out(FD_idata_out), .FD_iaddr_out(FD_iaddr_out));
    Control_unit c1(FD_idata_out,CU_we_out,CU_imm_out,CU_wer_out);
    regfile rf1(clk,rs1,rs2,rd_final,L_regdata_out,MW_wer_out,rv1,rv2,x31);
    ID_EX_reg de1(clk,flag,pc_replace,FD_idata_out[30],rs1,rs2,rd,FD_idata_out[14:12],FD_idata_out[6:0],CU_wer_out,rv1,rv2,x31,CU_we_out,CU_imm_out,FD_iaddr_out,DE_bit_th_out,DE_rs1_out,DE_rs2_out,DE_rd_out,DE_funct3_out,DE_op_out,DE_wer_out,DE_rv1_out,DE_rv2_out,DE_x31_out,DE_we_out,DE_imm_out,DE_iaddr_out,DE_flag_out);

    ALU a1(DE_funct3_out,DE_op_out, DE_bit_th_out,AU_rv1_in,AU_rv2_in,DE_we_out,DE_imm_out,DE_iaddr_out,EM_pc_replace_out,EM_flag_out,AU_regdata_out,AU_pc_out,AU_daddr_out,AU_we_out,AU_dwdata_out,pc_new,pc_replace,pc_JALR);
    EX_MEM_reg em1(clk,act,DE_op_out,DE_funct3_out,AU_daddr_out,AU_we_out,DE_wer_out,DE_rd_out,AU_regdata_out,AU_dwdata_out,pc_replace,flag,EM_op_out,EM_funct3_out,EM_daddr_out,EM_we_out,EM_wer_out,EM_rd_out,EM_regdata_out,EM_dwdata_out,EM_pc_replace_out,EM_flag_out); 
    dmem dm1(clk,EM_daddr_out,EM_dwdata_out,EM_we_out,DMEM_drdata_out);
    MEM_WB_reg mw1(clk,EM_op_out,EM_funct3_out,EM_daddr_out,EM_wer_out,EM_rd_out,EM_regdata_out,DMEM_drdata_out,MW_op_out,MW_funct3_out,MW_daddr_out,MW_wer_out,MW_rd_out,MW_regdata_out,MW_drdata_out);
    write_to_reg l1(MW_op_out,MW_funct3_out,MW_daddr_out,MW_drdata_out,MW_regdata_out,L_regdata_out);
	 WB_reg wb1(clk,MW_rd_out,L_regdata_out,MW_wer_out,WB_rd_out,WB_regdata_out,WB_wer_out);
endmodule


//Control Unit assigns the write enable signals and imm value
module Control_unit (
	 input [31:0] idata,  //Stores current Program counter value
    output reg [3:0] we,
    output reg signed [31:0] imm,
    output reg wer
);

    always@(*)     
    begin
            case(idata[6:0])
                7'b0110011:      //R type instructions
                begin
                    wer = 1;
                    we = 4'b0;
                end
                7'b0010011:     //I type instructions
                begin
                    imm = {{20{idata[31]}},idata[31:20]};
                    wer=1;
                    we=4'b0;
                end
                7'b0000011:     //L type instructions
                begin
                
                    imm = {{20{idata[31]}},idata[31:20]};
                    wer=1;
                    we=4'b0;
                end
                7'b0100011:     //S type instructions
                begin
                    imm = {{20{idata[31]}},idata[31:25],idata[11:7]};
                    wer=0;
                    case(idata[14:12])
                        3'b000: we = 4'b0001;
                        3'b001: we = 4'b0011;
                        3'b010: we = 4'b1111;   //Note that this is not the final we value. Only an intermediate
                    endcase
                end
				7'b1100011:		//B type instructions
				begin
				    imm = {{20{idata[31]}},idata[31],idata[7],idata[30:25],idata[11:8],1'b0};
					wer=0;
					we=4'b0;
				end
				7'b1100111:		//JALR instruction
				begin
				    imm = {{20{idata[31]}},idata[31:20]};
					wer = 1;
					we = 4'b0;				
				end
				7'b1101111:		//JAL instruction
				begin
					imm = {{11{idata[31]}},idata[31],idata[19:12],idata[20],idata[30:21],1'b0};
					wer = 1;
					we = 4'b0;
				end
				7'b0010111:		//AUIPC
				begin
					imm = {idata[31:12],12'b0};
					wer = 1;
					we = 4'b0;
				end
				7'b0110111:		//LUI
				begin
					imm = {idata[31:12],12'b0};
					wer=1;
					we=4'b0;
				end
			endcase
    end
endmodule

//ALU performs needed computations for different instruction types and returns value to the CPU which is used
//to update the PC or write to RF or write to DMEM.
module ALU (
    input [2:0] funct3,
    input [6:0] op,
    input bit_th,
    input [31:0] rv1,
    input [31:0] rv2,
    input [3:0] we, 
    input signed [31:0] imm,
    input [31:0] iaddr,
	 input pc_replace_old,
	 input flag_old,
    output reg [31:0] regdata,
    output reg [31:0] pc,
    output reg[31:0] daddr,
    output reg [3:0] we_final,
    output reg [31:0] dwdata,
	 output reg[31:0] pc_new,
	 output reg pc_replace,
	 output reg pc_JALR
);
	 wire [31:0] regdata_R, regdata_I;
	 wire [31:0] regdata_L, iaddr_val;
	 wire jump_flag;
	 initial begin
		pc_new=0;
		pc_replace=0;
		pc_JALR=0;
	 end
	 
    always@(*) 
    begin
		  pc_new=0;
		  pc_replace=0;
		  pc_JALR=0;
        case(op)
            7'b0110011:      //R type instructions
            begin
                regdata = regdata_R;
                we_final = we;
            end
            7'b0010011:     //I type instructions
            begin
                regdata = regdata_I;
                we_final = we;
            end
            7'b0000011:     //L type instructions
            begin
                daddr = rv1+imm;    
                we_final = we;
            end
            7'b0100011:     //S type instructions
            begin
                daddr = rv1+imm;
					case(funct3)
				 	3'b000: dwdata = {rv2[7:0],rv2[7:0],rv2[7:0],rv2[7:0]};
				 	3'b001: dwdata = {rv2[15:0],rv2[15:0]};
				 	3'b010: dwdata = rv2;
				 	endcase
                we_final = we<<daddr[1:0];
            end
				7'b1100011:		//B type instructions
				begin
			        pc_new = iaddr_val;
                 we_final = we;
					  pc_replace=(pc_replace_old|!flag_old)?0:jump_flag;
				end
				7'b1100111:		//JALR instruction
				begin
					regdata = iaddr+4;
					pc_new = (rv1+imm)&32'hfffffffe;
               we_final = we;
					pc_replace=(pc_replace_old|!flag_old)?0:1;
					pc_JALR=1;
				end
				7'b1101111:		//JAL instruction
				begin
					regdata = iaddr+4;
               pc_new = imm-12;
               we_final = we;
					pc_replace=(pc_replace_old|!flag_old)?0:1;
				end
				7'b0010111:		//AUIPC
				begin
					regdata = iaddr+imm;
               we_final = we;
				end
				7'b0110111:		//LUI
				begin
					regdata = imm;
               we_final = we;
				end
			endcase
    end

    //Instantiating modules from the computational block
    R_type r1(funct3,bit_th,rv1,rv2,regdata_R);   
    I_type i1(funct3,bit_th,rv1,imm,regdata_I);
	 B_type b1(funct3, iaddr, imm, rv1, rv2, iaddr_val,jump_flag);
	
endmodule


//Below are the modules corresponding to each instruction type. ALU uses these to get needed outputs.

//R type instructions
module R_type(
    input [2:0] funct3,
    input bit_th,
    input signed [31:0] in1,
    input signed [31:0] in2,
    output reg[31:0] out
);
    wire [31:0] tmp1;
    wire [31:0] tmp2;
    assign tmp1 = in1;
    assign tmp2 = in2;

    always @(*) 
    begin
    	case({bit_th,funct3})
        4'b0000:    out = in1+in2;          //add
        4'b1000:    out = in1-in2;          //sub
        4'b0001:    out = in1<<in2[4:0];	  //sll
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

//Module for immediate instructions
module I_type(
    input [2:0] funct3,
    input bit_th,
    input signed [31:0] in1,
    input signed [31:0] imm,
    output reg [31:0] out
);
    wire [31:0] tmp1;
    wire [11:0] tmp2;
    assign tmp1 = in1;
    assign tmp2 = imm;
    always @(*) 
    begin
    	case(funct3)
        3'b000: out = in1+imm;          //addi
        3'b010: out = in1<imm;          //slti
        3'b011: out= tmp1<tmp2;         //sltiu
        3'b100: out = in1 ^ imm;        //xori
        3'b110: out = in1 | imm;        //ori
        3'b111: out = in1 & imm;        //andi
        3'b001: out = in1<<imm[4:0];    //slli
        3'b101:
        begin
            if(bit_th)           //srli
               out = in1>>imm[4:0]; 
            else                    //srai
               out = in1>>>imm[4:0];
        end
        endcase
    end
endmodule



//Module for Store instructions - data to be written to reg file is generated by this module
module write_to_reg(
    input [6:0] op,
    input [2:0] funct3,
    input [31:0] daddr,
    input [31:0] drdata,
    input [31:0] MW_regdata_out,
    output reg[31:0] out
);
	reg [31:0] offset;

    always@(*)  //If this is a load instruction, then drdata has to be modified and written to reg file. Else, ALU output MW_regdata_out is used.
    begin
        if(op!=7'b0000011)  //If not load instruction
            out =  MW_regdata_out;     
        else
        begin
            offset = (daddr[1:0]<<3);  //offset = 8*rv1[1:0]-1 ,eg, 8*1=8, so drdata[15:8]
            case(funct3)
            3'b000: out = {{24{drdata[offset+7]}}, drdata[offset +: 8]};    //LB
            3'b001: out = {{16{drdata[offset+15]}}, drdata[offset +: 16]};  //LH
            3'b010: out = drdata;                                   		//LW
            3'b100: out = {24'b0, drdata[offset +: 8]};             		//LBU
            3'b101: out = {16'b0, drdata[offset +: 16]};            		//LHU
            endcase
        end
    end
endmodule


//Module for branch instructions - The new pc value is generated by this module
module B_type(
    input [2:0] funct3,
	input [31:0] iaddr,
	input signed [31:0] imm,
    input signed [31:0] in1, in2,
    output reg [31:0] out,
	 output reg jump_flag
);
	wire [31:0] tmp1, tmp2;
	assign tmp1 = in1;
	assign tmp2 = in2;
    always@(*)
    begin
		case(funct3)
        3'b000: 
		  begin
		  out = (in1==in2)? (imm-12) : (0);
		  jump_flag = (in1==in2)?1:0;
		  end
		  3'b001: 
		  begin
		  out = (in1!=in2)? (imm-12) : (0);
		  jump_flag = (in1!=in2)?1:0;
		  end
        3'b100: 
		  begin
		  out = (in1<in2)? (imm-12) : (0);
		  jump_flag = (in1<in2)?1:0;
		  end
        3'b101: 
		  begin
		  out = (in1>=in2)? (imm-12) : (0);
		  jump_flag = (in1>=in2)?1:0;
		  end
        3'b110: 
		  begin
		  out = (tmp1<tmp2)? (imm-12) : (0);
		  jump_flag = (tmp1<tmp2)?1:0;
		  end
        3'b111: 
		  begin
		  out = (tmp1>=tmp2)? (imm-12) : (0);
		  jump_flag = (tmp1>=tmp2)?1:0;
		  end
		endcase
	 end
endmodule
