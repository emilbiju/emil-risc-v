`timescale 1ns / 1ps

//IMEM module
module imem(
    input [31:0] iaddr,
    output [31:0] idata
);
    reg [31:0] i_arr[0:31];
    initial begin
		$readmemh("imem_ini_branch.hex.txt",i_arr);	//IMEM initialization file provided here
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
	 initial $readmemh("dmem_ini.hex.txt",m);
	 
	 assign add0 = (daddr & 32'hfffffffc)+ 32'h00000000;
	 assign add1 = (daddr & 32'hfffffffc)+ 32'h00000001;
	 assign add2 = (daddr & 32'hfffffffc)+ 32'h00000002;
	 assign add3 = (daddr & 32'hfffffffc)+ 32'h00000003;
	 
	 assign drdata = {m[add3], m[add2], m[add1], m[add0]};	//Data read from DMEM
	 
	 //Writing data to DMEM
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

// IF/ID register
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

// ID/EX register
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
	input branched,
	input [31:0] ignored_instr,
	input [31:0] FD_idata_out,
    //Register file outputs
	output reg [31:0] DE_idata_out,
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
	output reg DE_flag_out,
	output reg DE_branched_out,
	output reg [31:0] DE_ignored_instr_out
);

    always@(posedge clk)
    begin
		  
		if(flag==1)	//Flag is used to prevent the ID/EX register from receiving new values.
		//This is needed when there is a stall and part of the pipeline must be frozen for 1 clk cycle.
		begin
        DE_rs1_out = rs1_in;
        DE_rs2_out = rs2_in;
        DE_rd_out = rd_in;
        DE_funct3_out = funct3_in;
        DE_op_out = op;
        DE_imm_out = imm_in;
        DE_iaddr_out = iaddr_in;
        DE_wer_out =  wer_in&!pc_replace&flag;	//To set write control signals to 0 when it is detected that an instruction should not have been taken 
        DE_we_out = we_in&{!pc_replace&flag,!pc_replace&flag,!pc_replace&flag,!pc_replace&flag};
        DE_x31_out = x31_in;
        DE_rv1_out = rv1_in;
        DE_rv2_out = rv2_in;
        bit_th_out = bit_th_in;
		DE_flag_out = flag;
		DE_branched_out = branched;
		DE_ignored_instr_out = ignored_instr;
		DE_idata_out = FD_idata_out;
		end
    end
endmodule

// EX/MEM register
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
    	//Updating new values at each clk edge
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
    //Updating new values at each clk edge
        MW_funct3_out = EM_funct3_out;
        MW_op_out = EM_op_out;
        MW_daddr_out = EM_daddr_out;
        MW_wer_out = EM_wer_out;
        MW_rd_out = EM_rd_out;
        MW_regdata_out = EM_regdata_out;
        MW_drdata_out = DMEM_drdata_out;
    end
endmodule

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
	//Updating new values at each clk edge
		WB_rd_out = MW_rd_out;
		WB_regdata_out = L_regdata_out;
		WB_wer_out = MW_wer_out;
	end
endmodule

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
                for(i=0; i<31; i = i+1)	//Initialising RF
                    r[i]=i;
					 
            end
				always @(posedge clk) 
            begin
                if(wer && rd!=0)
                    r[rd] = regdata;	//Writing data at clock edge
            end
            //RF read outputs
            assign rv1 = r[rs1];	
            assign rv2 = r[rs2];
            assign x31 = r[31];	//Passing x31 value
            
endmodule

//CPU module
module pipeline_CPU(
    input clk,
    input reset,
    output reg [31:0] iaddr,  
    output [31:0] x31,
	output [31:0] L_regdata_out,
	output [31:0] EM_daddr_out,
    output [31:0] MW_drdata_out
);
	
	//All wire and reg declarations for the CPU.
	 wire [4:0] rs1,rs2,rd;
	 wire [31:0] rv1,rv2;
	 reg [0:0] tnt_tab[0:31];	//taken/not taken storage table
	 reg [31:0] targ_tab[0:31];  //target address storage table
	 integer i;
	
	 reg [2:0] progress;
	 wire [31:0] FD_idata_out;
     wire [31:0] FD_iaddr_out;
	 reg flag;
	 reg act;
	 wire pc_taken;
	 wire [31:0] pc_pred;
	 reg branched;
	 wire branch_success;
	 reg check_tab;
	 wire check_tab2;
	 reg [31:0] ignored_instr;
	 wire [31:0] pc_new;
	 wire pc_replace;
	 wire pc_JALR;
	 reg [31:0] iaddr_upd;
	 wire DE_branched_out;
	 wire [31:0] DE_ignored_instr_out;
     wire [4:0] DE_rs1_out;
     wire [4:0] DE_rs2_out;
     wire [4:0] DE_rd_out;
     wire DE_wer_out;
	 wire [31:0] CU_imm_out;
     wire [31:0] DE_rv1_out;
	 wire [31:0] DE_rv2_out;
     wire [31:0] DE_x31_out;
     wire [3:0] DE_we_out;
     wire signed [31:0] DE_imm_out;
     wire [31:0] DE_iaddr_out;
	 wire [31:0] DE_idata_out;
	 reg [31:0] AU_rv1_in;
	 reg [31:0] AU_rv2_in;
     wire [3:0] EM_we_out;
     wire EM_wer_out;
	 wire [6:0] EM_op_out;
     wire [4:0] EM_rd_out;
     wire [31:0] EM_regdata_out;
     wire [31:0] EM_dwdata_out;
     wire [31:0] MW_daddr_out;
     wire MW_wer_out;
     wire [4:0] MW_rd_out;
     wire [31:0] MW_regdata_out;
	 wire [31:0] DMEM_drdata_out;
	 wire [4:0] WB_rd_out;
	 wire [31:0] WB_regdata_out;
	 wire WB_wer_out;
	 
	 //Reset branch prediction table at start
	 initial begin
		for(i=0;i<32;i=i+1)
				begin
					tnt_tab[i]=0;
					targ_tab[i]=32'b0;
				end
	 end
	 
    always@(posedge clk)
        begin
				if(branch_success==0)	//If it is found that a taken branch was not supposed to be taken
				begin
					targ_tab[(DE_ignored_instr_out-4)>>2]=32'b0;
					tnt_tab[(DE_ignored_instr_out-4)>>2]=0;
				end
				ignored_instr=32'b0;
            	if(reset)    //Reset flags and iaddr
				begin
                	 iaddr = 0;
					 progress=0;
					 branched=0;
				end
				else
					begin
					//progress variable tracks how many clk cycles have been completed since start of program (upto 5 clk cycles)
					if(progress<5)
						progress=progress+1;
					else
						progress=progress;
					if(flag==0)
					begin
						iaddr = iaddr+4-iaddr_upd;	//iaddr update
						branched=branched;	
					end
					else if(pc_replace==0)
						begin
							branched=0;
							if(pc_taken==0)	//No branch intended to be taken in next clk cycle
								iaddr = iaddr+4;
							else
								begin
								ignored_instr = iaddr+4; //If it is found that branch was not supposed to be taken, the ignored_instr is loaded to iaddr
								iaddr = pc_pred; //Predicted branch is loaded to iaddr
								branched=1;	//Set branched to 1 if a branch is taken
								end
						end
					else if(pc_JALR==0)
						 begin
						 iaddr = pc_new; 
						 if(branch_success==1)
						 begin
						 //Updating branch prediction table if a branch is taken (setting T/NT flag to 1 and updating target address)
							 targ_tab[DE_iaddr_out>>2]=iaddr; 
							 tnt_tab[DE_iaddr_out>>2]=1;
						 end
						 end
					else
						 begin
						 iaddr = pc_new;
						 if(branch_success==1)
						 begin
						 //Updating branch prediction table if a branch is taken (setting T/NT flag to 1 and updating target address)
							 targ_tab[DE_iaddr_out>>2]=iaddr; 
							 tnt_tab[DE_iaddr_out>>2]=1;
						 end
						 end
				end
        end
	 
	 wire [31:0] idata;
     wire CU_wer_out;
     wire [3:0] CU_we_out,AU_we_out;
     wire [2:0] DE_funct3_out,EM_funct3_out,MW_funct3_out;
     wire [6:0] DE_op_out,MW_op_out;
     wire [31:0] AU_regdata_out,AU_pc_out,AU_daddr_out,AU_dwdata_out;
	 assign rs1 = FD_idata_out[19:15];
	 assign rs2 = FD_idata_out[24:20];
	 assign rd = FD_idata_out[11:7];
	 
	 wire [4:0] rd_final;
	 //If less than 4 clk cycles are complete, MW_rd_out will not have a valid value. So, we prevent garbage values from being written by setting rd to 0
	 assign rd_final = (progress<4)?0:MW_rd_out;
	 
	 always@(*)
	 begin
		iaddr_upd=32'b0;
		flag=1;
		act=1;
		
		if(EM_rd_out == DE_rs1_out && EM_wer_out == 1 && progress>=3)
		begin
			if(EM_op_out == 7'b0000011)	//for stalling if there is ALU after load.
			begin
				iaddr_upd = 4;
				flag=0;
				act=0;
			end
			else
				begin
				AU_rv1_in = EM_regdata_out;	//Forwarding from ALU
				end
		end
		else if(MW_rd_out == DE_rs1_out && MW_wer_out == 1 && progress>=4)
			begin
			AU_rv1_in = L_regdata_out;	//Forwarding from MEM stage
			end
		else if(WB_rd_out == DE_rs1_out && WB_wer_out == 1 && progress>=5)
			begin
			AU_rv1_in = WB_regdata_out;
			end
		else
			begin
			AU_rv1_in = DE_rv1_out;	//No forwarding case
			end
		if(EM_rd_out == DE_rs2_out && EM_wer_out == 1 && progress>=3 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
		begin
			if(EM_op_out == 7'b0000011)
			begin
				iaddr_upd = 4;
				flag=0;
				act=0;
			end
			else
			begin
			AU_rv2_in = EM_regdata_out; //Forwarding from ALU
			end
		end
		else if(MW_rd_out == DE_rs2_out && MW_wer_out == 1 && progress>=4 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
			begin
			AU_rv2_in = L_regdata_out; //Forwarding from MEM stage
			end
		else if(WB_rd_out == DE_rs2_out && WB_wer_out == 1 && progress>=5 && (DE_op_out[6:4]==3'b110 || DE_op_out[6:4]==3'b011 || DE_op_out[6:4]==3'b010))
			begin
			AU_rv2_in = WB_regdata_out;
			end
		else
			begin
			AU_rv2_in = DE_rv2_out;	//No forwarding case
			end
	 end
	
	 //pc_taken signals whether branch has to be taken at next clk edge. pc_pred stores target address to branch to.
	 assign pc_taken = (tnt_tab[iaddr>>2]==1)?1:0;
	 assign pc_pred = (tnt_tab[iaddr>>2]==1)?targ_tab[iaddr>>2]:0;
	
	 //Connections between modules
    imem im1(.iaddr(iaddr), .idata(idata));
    IF_ID_reg fd1(.clk(clk),.flag(flag),.pc_replace(pc_replace),.idata_in(idata), .iaddr_in(iaddr), .FD_idata_out(FD_idata_out), .FD_iaddr_out(FD_iaddr_out));
    Control_unit c1(FD_idata_out,CU_we_out,CU_imm_out,CU_wer_out);
    regfile rf1(clk,rs1,rs2,rd_final,L_regdata_out,MW_wer_out,rv1,rv2,x31);
    ID_EX_reg de1(clk,flag,pc_replace,FD_idata_out[30],rs1,rs2,rd,FD_idata_out[14:12],FD_idata_out[6:0],CU_wer_out,rv1,rv2,x31,CU_we_out,CU_imm_out,FD_iaddr_out,branched,ignored_instr,FD_idata_out,DE_idata_out,DE_bit_th_out,DE_rs1_out,DE_rs2_out,DE_rd_out,DE_funct3_out,DE_op_out,DE_wer_out,DE_rv1_out,DE_rv2_out,DE_x31_out,DE_we_out,DE_imm_out,DE_iaddr_out,DE_flag_out,DE_branched_out,DE_ignored_instr_out);

    ALU a1(DE_funct3_out,DE_op_out, DE_bit_th_out,AU_rv1_in,AU_rv2_in,DE_we_out,DE_imm_out,DE_iaddr_out,EM_pc_replace_out,EM_flag_out,DE_branched_out,DE_ignored_instr_out,DE_iaddr_out,AU_regdata_out,AU_pc_out,AU_daddr_out,AU_we_out,AU_dwdata_out,pc_new,pc_replace,pc_JALR,branch_success,check_tab2);
    EX_MEM_reg em1(clk,act,DE_op_out,DE_funct3_out,AU_daddr_out,AU_we_out,DE_wer_out,DE_rd_out,AU_regdata_out,AU_dwdata_out,pc_replace,flag,EM_op_out,EM_funct3_out,EM_daddr_out,EM_we_out,EM_wer_out,EM_rd_out,EM_regdata_out,EM_dwdata_out,EM_pc_replace_out,EM_flag_out); 
    dmem dm1(clk,EM_daddr_out,EM_dwdata_out,EM_we_out,DMEM_drdata_out);
    MEM_WB_reg mw1(clk,EM_op_out,EM_funct3_out,EM_daddr_out,EM_wer_out,EM_rd_out,EM_regdata_out,DMEM_drdata_out,MW_op_out,MW_funct3_out,MW_daddr_out,MW_wer_out,MW_rd_out,MW_regdata_out,MW_drdata_out);
    write_to_reg l1(MW_op_out,MW_funct3_out,MW_daddr_out,MW_drdata_out,MW_regdata_out,L_regdata_out);
	WB_reg wb1(clk,MW_rd_out,L_regdata_out,MW_wer_out,WB_rd_out,WB_regdata_out,WB_wer_out);
endmodule


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

//ALU module
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
	 input DE_branched_out,
	 input [31:0] DE_ignored_instr_out,
	 input [31:0] DE_iaddr_out,
     output reg [31:0] regdata,
     output reg [31:0] pc,
     output reg[31:0] daddr,
     output reg [3:0] we_final,
     output reg [31:0] dwdata,
	 output reg[31:0] pc_new,
	 output reg pc_replace,
	 output reg pc_JALR,
	 output reg branch_success,
	 output reg check_tab2
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
		  branch_success=1;
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
			        pc_new = DE_iaddr_out+iaddr_val+12;	//iaddr_val=offset-12
                 	we_final = we;
					  if(pc_replace_old)
							pc_replace=0;
					  //pc_replace=(pc_replace_old)?0:jump_flag;	 
					  //A|B|C|X -- If pc_replace corresponding to A was 1,which would only be understood at point X, then if B tries to branch, it should be avoided.
					  //Hence, if pc_replace_old=1, then branching due to B is avoided.
					  else
					  begin
							pc_replace=jump_flag;
						   if(DE_branched_out)
							begin
								if(jump_flag==1)
									pc_replace=0;
								else
								begin
									branch_success=0;
									pc_replace=1;
									check_tab2=1;
									pc_new = DE_ignored_instr_out;
							 	end
							 end
						end
				end
				7'b1100111:		//JALR instruction
				begin
					regdata = iaddr+4;
					pc_new = (rv1+imm)&32'hfffffffe;
               		we_final = we;
					pc_replace=(pc_replace_old)?0:!DE_branched_out;
					pc_JALR=!DE_branched_out;
				end
				7'b1101111:		//JAL instruction
				begin
					regdata = iaddr+4;
               		pc_new = DE_iaddr_out+imm;
               		we_final = we;
					pc_replace=(pc_replace_old)?0:!DE_branched_out;
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




//module for R type instructions
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
        3'b000: //BEQ
		  begin
		  out = (in1==in2)? (imm-12) : (0);
		  jump_flag = (in1==in2)?1:0;
		  end
		  3'b001: //BNE
		  begin
		  out = (in1!=in2)? (imm-12) : (0);
		  jump_flag = (in1!=in2)?1:0;
		  end
        3'b100:  //BLT
		  begin
		  out = (in1<in2)? (imm-12) : (0);
		  jump_flag = (in1<in2)?1:0;
		  end
        3'b101: //BGE
		  begin
		  out = (in1>=in2)? (imm-12) : (0);
		  jump_flag = (in1>=in2)?1:0;
		  end
        3'b110: //BLTU
		  begin
		  out = (tmp1<tmp2)? (imm-12) : (0);
		  jump_flag = (tmp1<tmp2)?1:0;
		  end
        3'b111: //BGEU
		  begin
		  out = (tmp1>=tmp2)? (imm-12) : (0);
		  jump_flag = (tmp1>=tmp2)?1:0;
		  end
		endcase
	 end
endmodule
