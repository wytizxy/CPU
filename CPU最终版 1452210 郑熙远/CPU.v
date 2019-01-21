`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 	Tongji	
// Engineer: 	Zheng Xiyuan
// 
// Create Date:    15:12:31 05/21/2016 
// Design Name: 
// Module Name:    CPU 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

//operation code
`define OP_ADDI		6'b001000
`define OP_ADDIU     6'b001001
`define OP_ANDI		6'b001100
`define OP_ORI		   6'b001101
`define OP_XORI   	6'b001110
`define OP_LW		   6'b100011
`define OP_SW		   6'b101011
`define OP_BEQ	    	6'b000100
`define OP_BNE   		6'b000101
`define OP_SLTI   	6'b001010
`define OP_SLTIU  	6'b001011
`define OP_LUI 		6'b001111
`define OP_J		   6'b000010
`define OP_JAL 		6'b000011
`define OP_ALUOp  	6'b000000
//ALU FUNCTION CODE
`define FUNC_ADD		6'b100000
`define FUNC_ADDU		6'b100001
`define FUNC_SUB		6'b100010
`define FUNC_SUBU   	6'b100011
`define FUNC_AND		6'b100100
`define FUNC_OR		6'b100101
`define FUNC_XOR   	6'b100110
`define FUNC_NOR		6'b100111
`define FUNC_SLT		6'b101010
`define FUNC_SLTU  	6'b101011
`define FUNC_SLL		6'b000000
`define FUNC_SRL		6'b000010
`define FUNC_SRA		6'b000011
`define FUNC_SLLV   	6'b000100
`define FUNC_SRLV   	6'b000110
`define FUNC_SRAV   	6'b000111
`define FUNC_JR   	6'b001000
//ALU CODE
`define ALU_ADD		4'b0000
`define ALU_SUB		4'b0001
`define ALU_AND		4'b0010
`define ALU_OR		   4'b0011
`define ALU_XOR      4'b0100
`define ALU_NOR		4'b0101 
`define ALU_SLT		4'b0110
`define ALU_SLTU     4'b0111
`define ALU_SLL	  	4'b1000
`define ALU_SRL		4'b1001
`define ALU_SRA		4'b1010
`define ALU_LUI		4'b1011
`define ALU_NONE	   4'b1100


module CPU(clk,rst,SevSeg,an);
			input 			clk,rst;
			output [6:0] SevSeg;
			output [7:0] an;
			wire [31:0] 	inst,ra,rb,p4,p8,npc,pc_in,addr,alua,alub,alu_out,alu_mem,
								res,mem,pc_value,data,value_out;		
			wire [3:0] 		aluc;				//ALU操作码
			wire [4:0]		reg_dust,wn;
			wire [1:0] 		pcsource;
			wire 				alu_zero,wmem,wreg,regrt,m2reg,shift,aluimm,jal,sext,alu_of;
			wire [5:0] 		shamt = {inst[10:6]};		//寄存器堆sa
			wire [31:0]		sa = {27'b0,inst[10:6]};
			PC				my_pc			(clk,rst,npc,pc_value);
			//imemip 			my_imem		(pc_value,inst);
			IMEM			my_imem		(pc_value,inst);
			DMEM 			my_dmem		(clk,alu_out,rb,wmem,mem);
			ctrl_unit 	cu 			(inst[31:26],inst[5:0],alu_zero,alu_of,wmem,wreg,regrt,
											 m2reg,aluc,shift,aluimm,pcsource,jal,sext);
			wire 				e = sext & inst[15];
			wire [15:0] 	imm = {16{e}};
			wire [31:0] 	immediate = {imm,inst[15:0]};
			wire [31:0] 	offset = {imm[13:0],inst[15:0],2'b00};
			cla32 		pcplus4 		(pc_value,32'h4,1'b0,p4);
			cla32 		br_adr 		(pc_value,offset,1'b0,addr);
			wire [31:0]		jpc = {p4[31:28],inst[25:0],2'b00};
			mux2_1x32	alu_b 		(rb,immediate,aluimm,alub);
			mux2_1x32 	alu_a			(ra,sa,shift,alua);
			mux2_1x32 	result		(alu_out,mem,m2reg,alu_mem);	
			cla32 		pcplus8 		(p4,32'h4,1'b0,p8);
			mux2_1x32	link 			(alu_mem,p8,jal,data);
			mux2_1x5		reg_wn		(inst[15:11],inst[20:16],regrt,reg_dust); 		
			mux4_1x32	nextpc		(p4,addr,ra,jpc,pcsource,npc);
			assign		wn = reg_dust | {5{jal}};		//jal: r31<--p4
			regfile		my_rf			(inst[25:21],inst[20:16],data,wn,wreg,clk,rst,ra,rb,value_out);
			alu			my_alu		(alua,alub,aluc,alu_out,alu_zero,alu_of);
			
			SevSegDsp	SevLight		(rst,clk,value_out,SevSeg,an);
endmodule

module alu(a,b,aluc,r,zero,overflow);
		input [31:0] a; // 32 位输入，操作数 1
		input [31:0] b;// 32 位输入，操作数 2
		input [3:0] aluc; // 4 位输入，控制 alu 的操作
		output[31:0] r; // 32 位输出，有 a b 经过 aluc 指定的操作生成
		output zero; // 0 标志位
		output overflow;
		reg [31:0] data_out;
		reg zero_out;
		reg carry_out;
		reg negative_out;
		reg overflow_out;
		reg signed [31:0]ua;
		reg signed [31:0]ub;
		assign r = data_out;
		assign zero = zero_out;
		assign carry = carry_out;
		assign negative = negative_out;
		assign overflow = overflow_out;
		always @(*) begin
		ua = a;
		ub = b;
		negative_out = 32'b0;
		carry_out = 1'b0;
		overflow_out = 1'b0;
		case (aluc)
		4'b0000: begin  //addu
					data_out = a + b;
					if((a[31]==1||b[31]==1)&&data_out[31]==0)
						carry_out = 1'b1;
					end
		4'b0010: begin   //add
					data_out = a + b;
					if((ua[31]==1&&ub[31]==1&&data_out[31]==0)||(ua[31]==0&&ub[31]==0&&data_out[31]==1))
						overflow_out = 1'b1;
						overflow_out = ua[31]&ub[31]&~data_out[31] | ~ua[31]&~ub[31]&data_out[31];
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b0001: begin   //subu
					data_out = a - b;
					if(a < b)
					carry_out = 1'b1;
					end
		4'b0011: begin   //sub
					data_out =ua - ub;
					//if((ua[31]==1&&ub[31]==0&&data_out[31]==0)||(ua[31]==0&&ub[31]==1&&data_out[31]==1))
						//overflow_out = 1'b1;
						overflow_out = ua[31]&~ub[31]&~data_out[31] | ~ua[31]&ub[31]&data_out[31];
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b0100: begin   //and
					data_out = a & b;
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b0101: begin   //or
					data_out = a | b;
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b0110: begin  // xor
					data_out = a ^ b;
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b0111: begin   //或非门 nor
					data_out = ~( a | b );
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b1000: begin  //lui
					data_out = {b[15:0] , 16'b0};
					end
		4'b1001: begin   //lui
					data_out = {b[15:0] , 16'b0};
					end
		4'b1011: begin   //slt
					data_out = (ua <ub) ? 32'b1 : 32'b0;
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b1010: begin  //sltu
					data_out = (a < b) ? 32'b1 : 32'b0;
					end
		4'b1100: begin   //sra
					carry_out = ub[a[4:0]-1];
					data_out = ub >>> a[4:0];
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b1110: begin   //sll/slr
					carry_out = b[32 - a[4:0]];
					data_out = b << a[4:0];
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b1111: begin //sll/slr
					carry_out = b[32 - a[4:0]];
					data_out = b << a[4:0];
					if(data_out < 32'b0) 
						negative_out = 32'b1;
					else
						negative_out = 32'b0;
					end
		4'b1101: begin   //srl
					carry_out = b[a[4:0]-1];
					data_out = b >> a[4:0];
					end
		endcase
		if (data_out == 32'b0) 
			zero_out = 1;
		else
			zero_out = 0;
		end
endmodule

module cla32(a,b,c,r);			
	input [31:0] a,b;
	input c;
	output [31:0] r;
	reg [31:0] r;
	always @ (a or b)
		if (c == 1'b0)begin
		r <= a + b;
		end
endmodule
		
module PC(	input clk, 									
				input rst, 							 									
				input [31 : 0] data_in, 				
				output reg [31 : 0] data_out);
	always @ (posedge clk or posedge rst)begin
			if(rst == 1)
				data_out <= 0;
			else
			begin
				data_out <= data_in;
			end
		end
endmodule

module ctrl_unit(op,func,z,overflow,wmem,wreg,regrt,m2reg,aluc,shift,aluimm,pcsource,jal,sext);
		input [5:0] op,func;
		input z,overflow;
		output wreg,regrt,jal,m2reg,shift,aluimm,sext,wmem;
		output [3:0] aluc;
		output [1:0] pcsource;
	
		wire r_type  = ~|op;
		wire i_add   = r_type& func[5]&~func[4]&~func[3]&~func[2]&~func[1]&~func[0];
		wire i_addu  = r_type& func[5]&~func[4]&~func[3]&~func[2]&~func[1]& func[0];
		wire i_sub   = r_type& func[5]&~func[4]&~func[3]&~func[2]& func[1]&~func[0];
		wire i_subu  = r_type& func[5]&~func[4]&~func[3]&~func[2]& func[1]& func[0];
		wire i_and   = r_type& func[5]&~func[4]&~func[3]& func[2]&~func[1]&~func[0];
		wire i_or    = r_type& func[5]&~func[4]&~func[3]& func[2]&~func[1]& func[0];
		wire i_xor   = r_type& func[5]&~func[4]&~func[3]& func[2]& func[1]&~func[0];
		wire i_slt   = r_type& func[5]&~func[4]& func[3]&~func[2]& func[1]&~func[0];
		wire i_sltu  = r_type& func[5]&~func[4]& func[3]&~func[2]& func[1]& func[0];
		wire i_nor   = r_type& func[5]&~func[4]&~func[3]& func[2]& func[1]& func[0];
		wire i_sll   = r_type&~func[5]&~func[4]&~func[3]&~func[2]&~func[1]&~func[0];
		wire i_srl   = r_type&~func[5]&~func[4]&~func[3]&~func[2]& func[1]&~func[0];
		wire i_sra   = r_type&~func[5]&~func[4]&~func[3]&~func[2]& func[1]& func[0];
		wire i_sllv  = r_type&~func[5]&~func[4]&~func[3]& func[2]&~func[1]&~func[0];
		wire i_srlv  = r_type&~func[5]&~func[4]&~func[3]& func[2]& func[1]&~func[0];
		wire i_srav  = r_type&~func[5]&~func[4]&~func[3]& func[2]& func[1]& func[0];
		wire i_jr    = r_type&~func[5]&~func[4]& func[3]&~func[2]&~func[1]&~func[0];
	
		wire i_addi  = ~op[5]&~op[4]& op[3]&~op[2]&~op[1]&~op[0];
		wire i_addiu = ~op[5]&~op[4]& op[3]&~op[2]&~op[1]& op[0];
		wire i_andi  = ~op[5]&~op[4]& op[3]& op[2]&~op[1]&~op[0];
		wire i_ori   = ~op[5]&~op[4]& op[3]& op[2]&~op[1]& op[0];
		wire i_xori  = ~op[5]&~op[4]& op[3]& op[2]& op[1]&~op[0];
		wire i_lw    =  op[5]&~op[4]&~op[3]&~op[2]& op[1]& op[0];
		wire i_sw    =  op[5]&~op[4]& op[3]&~op[2]& op[1]& op[0];
		wire i_beq   = ~op[5]&~op[4]&~op[3]& op[2]&~op[1]&~op[0];
		wire i_bne   = ~op[5]&~op[4]&~op[3]& op[2]&~op[1]& op[0];
		wire i_slti  = ~op[5]&~op[4]& op[3]&~op[2]& op[1]&~op[0];
		wire i_sltiu = ~op[5]&~op[4]& op[3]&~op[2]& op[1]& op[0];
		wire i_lui   = ~op[5]&~op[4]& op[3]& op[2]& op[1]& op[0];
		wire i_j     = ~op[5]&~op[4]&~op[3]&~op[2]& op[1]&~op[0];
		wire i_jal   = ~op[5]&~op[4]&~op[3]&~op[2]& op[1]& op[0];
	 
		assign wreg  = (i_add & ~overflow)  | i_addu | (i_sub & ~overflow)  | i_subu | i_and  | 
							i_or   | i_xor  | i_nor  | i_slt  | i_sltu | 
							i_sll  | i_srl  | i_sra  | i_sllv | i_srlv | 
							i_srav | i_addi | i_addiu| i_andi | i_ori  |	 
							i_xori | i_lw   | i_slti | i_sltiu| i_lui  | 
							i_jal	 ;
		
		assign regrt = i_addi | i_addiu| i_andi | i_ori  | i_xori | 
							i_lw   | i_sltiu| i_sltiu| i_lui ;

		assign jal = i_jal;	

		assign m2reg = i_lw;

		assign shift = i_sll | i_srl | i_sra;

		assign aluimm = i_addi | i_addiu | i_andi | i_ori   | i_xori |
							 i_lw	  | i_sw	   | i_slti | i_sltiu | i_lui;

		assign sext = i_addi | i_addiu| i_lw   | i_sw | i_beq |
						  i_bne  | i_slti | i_sltiu;

		assign aluc[3]	=  i_slt  | i_sltu | i_sll  | i_srl  | i_sra  |
								i_sllv | i_srlv | i_srav | i_slti | i_sltiu|
								i_lui;

		assign aluc[2] = i_and  | i_or   | i_xor  | i_nor  | i_sll  |
								i_srl  | i_sra  | i_sllv | i_srlv | i_srav |
								i_andi | i_ori  | i_xori;

		assign aluc[1] = i_add  | i_sub  | i_xor  | i_nor  | i_slt  |
								i_sltu | i_sll  | i_sllv | i_addi | i_xori |
								i_lw   | i_sw   | i_beq  | i_bne  | i_slti |
								i_sltiu;

		assign aluc[0] = i_sub  | i_subu | i_or   | i_nor  | i_slt  |
								i_srl  | i_srlv | i_ori  | i_slti | i_beq  |
								i_bne;

		assign wmem	= i_sw;

		assign pcsource[1] = i_jr | i_j | i_jal;

		assign pcsource[0] = (i_beq & z) | (i_bne & ~z) | i_j | i_jal;
	
endmodule



module mux4_1x32(a,b,c,d,option,r);	//四选一选择器 a/b/c/d为输入，option为选择，r为输出
		input[31:0] a;
		input[31:0] b;
		input[31:0] c;
		input[31:0] d;
		input[1:0] option;
		output[31:0] r;
		reg [31:0] r;
		always @ (option or a or b or c or d)
			if (option == 2'b00) r <= a;
			else if (option == 2'b01) r <= b;
			else if (option == 2'b10) r <= c;
			else r <= d;

endmodule

module mux2_1x5(a,b,option,r);	//二选一选择器 a/b为输入，option为选择，r为输出
		input[4:0] a;
		input[4:0] b;
		input option;
		output[4:0] r;
		reg [4:0] r;
		always @ (option or a or b)
			if (option == 0) r <= a;
			else r <= b;

endmodule

module mux2_1x32(a,b,option,r);	//二选一选择器 a/b为输入，option为选择，r为输出
	 input[31:0] a;
		input[31:0] b;
		input option;
		output[31:0] r;
		reg [31:0] r;
		always @ (option or a or b)
			if (option == 0) r <= a;
			else r <= b;

endmodule

module IMEM(addr,instr);	
		input [31:0] addr;
		output [31:0] instr;
		reg [31:0] iram [1023:0];//先这么多，应该够用
		
		
		initial 
			begin
				$readmemh("text.txt",iram);
			end
				
		assign instr = iram[addr[31:2]];
endmodule

module DMEM(clk,addr,data,we,mem_out);
		input [31:0] addr;
		input [31:0] data;
		input clk;
		input we;				//写信号
		output [31:0] mem_out;
		reg [31:0] drom [0:2047];
		assign mem_out = drom[addr[6:2]];
		always @ (posedge clk)
		begin
				if (we == 1)
						drom[addr[6:2]] <= data;
						
		end		 
endmodule

module regfile(raddr1,raddr2,wdata,waddr,we,clk,rst,rdata1,rdata2,value_out);
	input clk; //寄存器组时钟信号，下降沿写入数据（注意：pc 为上升沿，此为下降沿）
	input rst; //reset 信号，reset 有效时全部寄存器置零
	input we; //写有效信号，we 有效时寄存器才能被写入
	input [4:0] raddr1; //所需读取的寄存器的地址
	input [4:0] raddr2; //所需读取的寄存器的地址
	input [4:0] waddr; //写寄存器的地址
	input [31:0] wdata; //写寄存器数据
	output [31:0] rdata1; //raddr1 所对应寄存器的数据，只要有raddr1 的输入即输出相应数据
	output [31:0] rdata2; //raddr2 所对应寄存器的数据，只要有raddr2 的输入即输出相应数据
	output [31:0] value_out;
	wire [31:0] reg_s;
		wire [31:0] sz[31:0];
		assign value_out = sz[9];
		decoder decd(.ena(we),.data_in(waddr),.data_out(reg_s));
		pcreg reg0(.clk(clk),.rst(rst),.enr(1'b0),.data_in(wdata),.data_out(sz[0]));
		pcreg reg1(.clk(clk),.rst(rst),.enr(reg_s[1]),.data_in(wdata),.data_out(sz[1]));	 
		pcreg reg2(.clk(clk),.rst(rst),.enr(reg_s[2]),.data_in(wdata),.data_out(sz[2]));	 
		pcreg reg3(.clk(clk),.rst(rst),.enr(reg_s[3]),.data_in(wdata),.data_out(sz[3]));	 
		pcreg reg4(.clk(clk),.rst(rst),.enr(reg_s[4]),.data_in(wdata),.data_out(sz[4]));	 
		pcreg reg5(.clk(clk),.rst(rst),.enr(reg_s[5]),.data_in(wdata),.data_out(sz[5]));	 
		pcreg reg6(.clk(clk),.rst(rst),.enr(reg_s[6]),.data_in(wdata),.data_out(sz[6]));	 
		pcreg reg7(.clk(clk),.rst(rst),.enr(reg_s[7]),.data_in(wdata),.data_out(sz[7]));	 
		pcreg reg8(.clk(clk),.rst(rst),.enr(reg_s[8]),.data_in(wdata),.data_out(sz[8]));	 
		pcreg reg9(.clk(clk),.rst(rst),.enr(reg_s[9]),.data_in(wdata),.data_out(sz[9]));	 
		pcreg reg10(.clk(clk),.rst(rst),.enr(reg_s[10]),.data_in(wdata),.data_out(sz[10]));	 
		pcreg reg11(.clk(clk),.rst(rst),.enr(reg_s[11]),.data_in(wdata),.data_out(sz[11]));	 
		pcreg reg12(.clk(clk),.rst(rst),.enr(reg_s[12]),.data_in(wdata),.data_out(sz[12]));	 
		pcreg reg13(.clk(clk),.rst(rst),.enr(reg_s[13]),.data_in(wdata),.data_out(sz[13]));	 
		pcreg reg14(.clk(clk),.rst(rst),.enr(reg_s[14]),.data_in(wdata),.data_out(sz[14]));	 
		pcreg reg15(.clk(clk),.rst(rst),.enr(reg_s[15]),.data_in(wdata),.data_out(sz[15]));	 
		pcreg reg16(.clk(clk),.rst(rst),.enr(reg_s[16]),.data_in(wdata),.data_out(sz[16]));	 
		pcreg reg17(.clk(clk),.rst(rst),.enr(reg_s[17]),.data_in(wdata),.data_out(sz[17]));	 
		pcreg reg18(.clk(clk),.rst(rst),.enr(reg_s[18]),.data_in(wdata),.data_out(sz[18]));	 
		pcreg reg19(.clk(clk),.rst(rst),.enr(reg_s[19]),.data_in(wdata),.data_out(sz[19]));	 
		pcreg reg20(.clk(clk),.rst(rst),.enr(reg_s[20]),.data_in(wdata),.data_out(sz[20]));	 
		pcreg reg21(.clk(clk),.rst(rst),.enr(reg_s[21]),.data_in(wdata),.data_out(sz[21]));	 
		pcreg reg22(.clk(clk),.rst(rst),.enr(reg_s[22]),.data_in(wdata),.data_out(sz[22]));	 
		pcreg reg23(.clk(clk),.rst(rst),.enr(reg_s[23]),.data_in(wdata),.data_out(sz[23]));	 
		pcreg reg24(.clk(clk),.rst(rst),.enr(reg_s[24]),.data_in(wdata),.data_out(sz[24]));	
		pcreg reg25(.clk(clk),.rst(rst),.enr(reg_s[25]),.data_in(wdata),.data_out(sz[25]));	 
		pcreg reg26(.clk(clk),.rst(rst),.enr(reg_s[26]),.data_in(wdata),.data_out(sz[26]));	 
		pcreg reg27(.clk(clk),.rst(rst),.enr(reg_s[27]),.data_in(wdata),.data_out(sz[27]));	 
		pcreg reg28(.clk(clk),.rst(rst),.enr(reg_s[28]),.data_in(wdata),.data_out(sz[28]));	 
		pcreg reg29(.clk(clk),.rst(rst),.enr(reg_s[29]),.data_in(wdata),.data_out(sz[29]));	 
		pcreg reg30(.clk(clk),.rst(rst),.enr(reg_s[30]),.data_in(wdata),.data_out(sz[30]));	 
		pcreg reg31(.clk(clk),.rst(rst),.enr(reg_s[31]),.data_in(wdata),.data_out(sz[31]));
		mux mux1(.choice(raddr1),
					.data1(sz[0]),
					.data2(sz[1]),
					.data3(sz[2]),
					.data4(sz[3]),
					.data5(sz[4]),
					.data6(sz[5]),
					.data7(sz[6]),
					.data8(sz[7]),
					.data9(sz[8]),
					.data10(sz[9]),
					.data11(sz[10]),
					.data12(sz[11]),
					.data13(sz[12]),
					.data14(sz[13]),
					.data15(sz[14]),
					.data16(sz[15]),
					.data17(sz[16]),
					.data18(sz[17]),
					.data19(sz[18]),
					.data20(sz[19]),
					.data21(sz[20]),
					.data22(sz[21]),
					.data23(sz[22]),
					.data24(sz[23]),
					.data25(sz[24]),
					.data26(sz[25]),
					.data27(sz[26]),
					.data28(sz[27]),
					.data29(sz[28]),
					.data30(sz[29]),
					.data31(sz[30]),
					.data32(sz[31]),
					.out(rdata1));
		mux mux2(.choice(raddr2),
					.data1(sz[0]),
					.data2(sz[1]),
					.data3(sz[2]),
					.data4(sz[3]),
					.data5(sz[4]),
					.data6(sz[5]),
					.data7(sz[6]),
					.data8(sz[7]),
					.data9(sz[8]),
					.data10(sz[9]),
					.data11(sz[10]),
					.data12(sz[11]),
					.data13(sz[12]),
					.data14(sz[13]),
					.data15(sz[14]),
					.data16(sz[15]),
					.data17(sz[16]),
					.data18(sz[17]),
					.data19(sz[18]),
					.data20(sz[19]),
					.data21(sz[20]),
					.data22(sz[21]),
					.data23(sz[22]),
					.data24(sz[23]),
					.data25(sz[24]),
					.data26(sz[25]),
					.data27(sz[26]),
					.data28(sz[27]),
					.data29(sz[28]),
					.data30(sz[29]),
					.data31(sz[30]),
					.data32(sz[31]),
					.out(rdata2)); 
endmodule


module decoder(
				input [4:0] data_in,
				input ena,
				output [31:0] data_out
				);
		reg[31:0] data_temp;
		assign data_out = ~data_temp;
			always @ (ena or data_in) begin
				if (ena == 1)
					case (data_in)
					5'b00000:
							data_temp = 32'b11111111111111111111111111111110;
					5'b00001:
							data_temp = 32'b11111111111111111111111111111101;
					5'b00010:
							data_temp = 32'b11111111111111111111111111111011;
					5'b00011:
							data_temp = 32'b11111111111111111111111111110111;
					5'b00100:
							data_temp = 32'b11111111111111111111111111101111;
					5'b00101:
							data_temp = 32'b11111111111111111111111111011111;
					5'b00110:
							data_temp = 32'b11111111111111111111111110111111;
					5'b00111:
							data_temp = 32'b11111111111111111111111101111111;
					5'b01000:
							data_temp = 32'b11111111111111111111111011111111;
					5'b01001:
							data_temp = 32'b11111111111111111111110111111111;
					5'b01010:
							data_temp = 32'b11111111111111111111101111111111;
					5'b01011:
							data_temp = 32'b11111111111111111111011111111111;
					5'b01100:
							data_temp = 32'b11111111111111111110111111111111;
					5'b01101:
							data_temp = 32'b11111111111111111101111111111111;
					5'b01110:
							data_temp = 32'b11111111111111111011111111111111;
					5'b01111:
							data_temp = 32'b11111111111111110111111111111111;
					5'b10000:
							data_temp = 32'b11111111111111101111111111111111;
					5'b10001:
							data_temp = 32'b11111111111111011111111111111111;
					5'b10010:
							data_temp = 32'b11111111111110111111111111111111;
					5'b10011:
							data_temp = 32'b11111111111101111111111111111111;
					5'b10100:
							data_temp = 32'b11111111111011111111111111111111;
					5'b10101:
							data_temp = 32'b11111111110111111111111111111111;
					5'b10110:
							data_temp = 32'b11111111101111111111111111111111;
					5'b10111:
							data_temp = 32'b11111111011111111111111111111111;
					5'b11000:
							data_temp = 32'b11111110111111111111111111111111;
					5'b11001:
							data_temp = 32'b11111101111111111111111111111111;
					5'b11010:
							data_temp = 32'b11111011111111111111111111111111;
					5'b11011:
							data_temp = 32'b11110111111111111111111111111111;
					5'b11100:
							data_temp = 32'b11101111111111111111111111111111;
					5'b11101:
							data_temp = 32'b11011111111111111111111111111111;
					5'b11110:
							data_temp = 32'b10111111111111111111111111111111;
					5'b11111:
							data_temp = 32'b01111111111111111111111111111111;
					endcase
				else
					data_temp= 32'b11111111111111111111111111111111;
			end
endmodule

module pcreg(
			input clk, 									
			input rst, 																					
			input enr, 									
			input [31 : 0] data_in, 				
			output reg [31 : 0] data_out 			
			);	always @ (negedge clk)begin
			if(rst == 1)
				data_out <= 0;
			else
				if(enr == 1)
					data_out <= data_in;
		end
endmodule

module mux(
		 input [31:0] data1,
		 input [31:0] data2,
		 input [31:0] data3,
		 input [31:0] data4,
		 input [31:0] data5,
		 input [31:0] data6,
		 input [31:0] data7,
		 input [31:0] data8,
		 input [31:0] data9,
		 input [31:0] data10,
		 input [31:0] data11,
		 input [31:0] data12,
		 input [31:0] data13,
		 input [31:0] data14,
		 input [31:0] data15,
		 input [31:0] data16,
		 input [31:0] data17,
		 input [31:0] data18,
		 input [31:0] data19,
		 input [31:0] data20,
		 input [31:0] data21,
		 input [31:0] data22,
		 input [31:0] data23,
		 input [31:0] data24,
		 input [31:0] data25,
		 input [31:0] data26,
		 input [31:0] data27,
		 input [31:0] data28,
		 input [31:0] data29,
		 input [31:0] data30,
		 input [31:0] data31,
		 input [31:0] data32,
		 input [4:0] choice,
		 output[31:0] out
		);
			reg[31:0] dataout;
			assign out = dataout;
			always@(*)
				begin
					case(choice)
						5'b00000:
								dataout = data1;
						5'b00001:
								dataout = data2;
						5'b00010:
								dataout = data3;
						5'b00011:
								dataout = data4;
					   5'b00100:
								dataout = data5;
						5'b00101:
								dataout = data6;
						5'b00110:
								dataout = data7;
						5'b00111:
								dataout = data8;
						5'b01000:
								dataout = data9;
						5'b01001:
								dataout = data10;
						5'b01010:
								dataout = data11;
						5'b01011:
								dataout = data12;
					   5'b01100:
								dataout = data13;
						5'b01101:
								dataout = data14;
						5'b01110:
								dataout = data15;
						5'b01111:
								dataout = data16; 
						5'b10000:
								dataout = data17;
						5'b10001:
								dataout = data18; 
						5'b10010:
								dataout = data19;	
						5'b10011:
								dataout = data20;	
						5'b10100:
								dataout = data21;
						5'b10101:
								dataout = data22;
						5'b10110:
								dataout = data23;
						5'b10111:
								dataout = data24;
						5'b11000:
								dataout = data25;
						5'b11001:
								dataout = data26; 
						5'b11010:
								dataout = data27;	
						5'b11011:
								dataout = data28;
						5'b11100:
								dataout = data29;
						5'b11101:
								dataout = data30;
						5'b11110:
								dataout = data31;	
						5'b11111:
								dataout = data32;						  
					endcase
				end
endmodule

module SevSegDsp(
		input wire rst,
		input wire clk,
		input [31:0] num_in,
		output reg [6:0] SevSeg,
		output wire [7:0] an
    );
		
		reg [3:0]t = 0;
		//reg [31:0] num_in;
		reg [3:0] num_out [7:0];
		reg i,clkout;
		reg [31:0]cnt;
		reg [2:0]scan_cnt;
		reg [3:0]num;
		reg [7:0]DIG_r ;
		reg [31:0] num0;
		parameter  period= 100000;
		assign an =~DIG_r;
		
		always @ (clk)begin
			num_out[6] = num_in[31:28];
			num_out[5] = num_in[27:24];
			num_out[4] = num_in[23:20];
			num_out[3] = num_in[19:16];
			num_out[2] = num_in[15:12];
			num_out[1] = num_in[11:8];
			num_out[0] = num_in[7:4];
			num_out[7] = num_in[3:0];
		end
		
		
		always @ (posedge clk or posedge rst) begin		//分频50Hz
			if(rst)
				cnt <= 0;
			else begin
				cnt <= cnt + 1;
				if (cnt == (period >> 1) - 1)
					clkout <= #1 1'b1;
				else if (cnt == period - 1)
					begin
						clkout <= #1 1'b0;
						cnt <= #1 1'b0;
					end
				end
			end
		
		always @(posedge clkout or posedge rst)begin 
			if (rst)begin
				scan_cnt <= 0 ;
				num <= 0;
				t <= 0;
				end
			else  begin
				scan_cnt <= scan_cnt + 1;
				num <= num_out[t];
				t <= t + 1;
				if(scan_cnt == 3'b111) 
					//t <= 0;
					scan_cnt <= 0;
				end 
			end
		
		always @(scan_cnt)         //数码管选择
			begin 
				case (scan_cnt)    
					3'b000 : DIG_r <= 8'b00000001;    
					3'b001 : DIG_r <= 8'b00000010;    
					3'b010 : DIG_r <= 8'b00000100;    
					3'b011 : DIG_r <= 8'b00001000;    
					3'b100 : DIG_r <= 8'b00010000;    
					3'b101 : DIG_r <= 8'b00100000;    
					3'b110 : DIG_r <= 8'b01000000;     
					3'b111 : DIG_r <= 8'b10000000;    
					default :DIG_r <= 8'b00000000;    
				endcase
			end
		
		always @ (num) //译码
			begin
				case (num)
					4'b0000: SevSeg = 7'b0000001;
					4'b0001:	SevSeg = 7'b1001111;
					4'b0010:	SevSeg = 7'b0010010;
					4'b0011: SevSeg = 7'b0000110;
					4'b0100: SevSeg = 7'b1001100;
					4'b0101: SevSeg = 7'b0100100;
					4'b0110: SevSeg = 7'b0100000;
					4'b0111: SevSeg = 7'b0001111;
					4'b1000: SevSeg = 7'b0000000;
					4'b1001: SevSeg = 7'b0000100;
					4'b1010: SevSeg = 7'b0001000;
					4'b1011: SevSeg = 7'b1100000;
					4'b1100: SevSeg = 7'b1110010;
					4'b1101: SevSeg = 7'b1000010;
					4'b1110: SevSeg = 7'b0110000;
					4'b1111: SevSeg = 7'b0111000;
				endcase
			end
endmodule
