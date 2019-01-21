`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 	Tongji
// Engineer:	1452210 Zheng Xiyuan
//
// Create Date:   19:22:35 06/05/2016
// Design Name:   CPU
// Module Name:   D:/Projects/MIPS/codes/Ex10/cpu_tb.v
// Project Name:  Ex10
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: CPU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module cpu_tb;

	// Inputs
	
	reg clk;
	reg rst;

	// Outputs

	// Instantiate the Unit Under Test (UUT)
	CPU uut (
		.clk(clk), 
		.rst(rst)
	);
		integer file_output;
		integer counter = 0;
	initial begin
		// Initialize Inputs
		file_output = $fopen("result_.txt");
	
		clk = 0;
		rst = 1;      //必须初始化一次，使pc获得初始值0
		
		
		// Wait 100 ns for global reset to finish
		#10;
		rst=0;
	
		// Add stimulus here
		

		end
   
		always #10 clk = ~clk;   //为CPU提供时序
		
		always @(posedge clk)
		begin
		counter = counter + 1;    //在后仿真中，不能使用文件输出。那如何检查结果是否正确？
											//为节约时间，以前仿真举例
		
		$fdisplay(file_output,"regfiles0 = %h",uut.my_rf.sz[0]);
		$fdisplay(file_output,"regfiles1 = %h",uut.my_rf.sz[1]);
		$fdisplay(file_output,"regfiles2 = %h",uut.my_rf.sz[2]);
		$fdisplay(file_output,"regfiles3 = %h",uut.my_rf.sz[3]);
		$fdisplay(file_output,"regfiles4 = %h",uut.my_rf.sz[4]);
		$fdisplay(file_output,"regfiles5 = %h",uut.my_rf.sz[5]);
		$fdisplay(file_output,"regfiles6 = %h",uut.my_rf.sz[6]);
		$fdisplay(file_output,"regfiles7 = %h",uut.my_rf.sz[7]);
		$fdisplay(file_output,"regfiles8 = %h",uut.my_rf.sz[8]);
		$fdisplay(file_output,"regfiles9 = %h",uut.my_rf.sz[9]);
		$fdisplay(file_output,"regfiles10 = %h",uut.my_rf.sz[10]);
		$fdisplay(file_output,"regfiles11 = %h",uut.my_rf.sz[11]);
		$fdisplay(file_output,"regfiles12 = %h",uut.my_rf.sz[12]);
		$fdisplay(file_output,"regfiles13 = %h",uut.my_rf.sz[13]);
		$fdisplay(file_output,"regfiles14 = %h",uut.my_rf.sz[14]);
		$fdisplay(file_output,"regfiles15 = %h",uut.my_rf.sz[15]);
		$fdisplay(file_output,"regfiles16 = %h",uut.my_rf.sz[16]);
		$fdisplay(file_output,"regfiles17 = %h",uut.my_rf.sz[17]);
		$fdisplay(file_output,"regfiles18 = %h",uut.my_rf.sz[18]);
		$fdisplay(file_output,"regfiles19 = %h",uut.my_rf.sz[19]);
		$fdisplay(file_output,"regfiles20 = %h",uut.my_rf.sz[20]);
		$fdisplay(file_output,"regfiles21 = %h",uut.my_rf.sz[21]);
		$fdisplay(file_output,"regfiles22 = %h",uut.my_rf.sz[22]);
		$fdisplay(file_output,"regfiles23 = %h",uut.my_rf.sz[23]);
		$fdisplay(file_output,"regfiles24 = %h",uut.my_rf.sz[24]);
		$fdisplay(file_output,"regfiles25 = %h",uut.my_rf.sz[25]);
		$fdisplay(file_output,"regfiles26 = %h",uut.my_rf.sz[26]);
		$fdisplay(file_output,"regfiles27 = %h",uut.my_rf.sz[27]);
		$fdisplay(file_output,"regfiles28 = %h",uut.my_rf.sz[28]);
		$fdisplay(file_output,"regfiles29 = %h",uut.my_rf.sz[29]);
		$fdisplay(file_output,"regfiles30 = %h",uut.my_rf.sz[30]);
		$fdisplay(file_output,"regfiles31 = %h",uut.my_rf.sz[31]);		
		
		//$fdisplay(file_output,"instr = %h",uut.my_imem.spo);
		$fdisplay(file_output,"instr = %h",uut.my_imem.instr);
		$fdisplay(file_output,"pc = %h",uut.my_pc.data_out);
		
		//$fdisplay(file_output,"value_out = %h",uut.my_dmem.data_out);
		//$fdisplay(file_output,"value_out = %h",uut.SevLight.num_in);
		//$fdisplay(file_output,"alua = %h",uut.alu_a.r);
		//$fdisplay(file_output,"alub = %h",uut.alu_b.r);
		//$fdisplay(file_output,"aluc = %h",uut.cu.aluc);
		//$fdisplay(file_output,"alur = %h",uut.my_alu.r);
		//$fdisplay(file_output,"imm = %h",uut.immediate);
		//$fdisplay(file_output,"data = %h",uut.link.r);
		//$fdisplay(file_output,"wreg = %h",uut.cu.wreg);
		//$fdisplay(file_output,"reg_dust = %h",uut.reg_wn.r);
		//$fdisplay(file_output,"result_a = %h",uut.result.a);
		//$fdisplay(file_output,"result_b = %h",uut.result.b);
		//$fdisplay(file_output,"result_op = %h",uut.result.option);
		//$fdisplay(file_output,"result_r = %h",uut.result.r);
		//$fdisplay(file_output,"jal = %h",uut.link.option);
		//$fdisplay(file_output,"rf_datain = %h",uut.link.r);
		//$fdisplay(file_output,"reg_rb = %h",uut.my_rf.rdata2);
		//$fdisplay(file_output,"DMEM_we = %h",uut.my_dmem.we);
		//$fdisplay(file_output,"DMEM_data = %h",uut.my_dmem.data);
		//$fdisplay(file_output,"DMEM_out = %h",uut.my_dmem.mem_out);
		//$fdisplay(file_output,"offset = %h",uut.br_adr.b);
		//$fdisplay(file_output,"offset = %h",uut.br_adr.b);
		//$fdisplay(file_output,"pcsource = %h",uut.cu.pcsource);
		//$fdisplay(file_output,"nextpc = %h",uut.nextpc.r);
		end
      
endmodule

