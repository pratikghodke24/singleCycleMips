`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ghodke Pratik Pravin
// 
// Create Date:    15:31:28 02/26/2020 
// Design Name: 
// Module Name:     
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: MIPS single cycle
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////




module regfile(input clk,
							input regWrite, 
							input [4:0] rs, input [4:0] rt, input [4:0] rd,
							input [31:0] WD,
					      output [31:0] RD1, output [31:0] RD2);
					
			reg [31:0] rf [31:0];
					
			always @(posedge clk) begin
				if(regWrite == 1'b1) begin
						rf[rd] <= WD;
				end
			end
			
			assign RD1 = (rs!=0)?rf[rs]:0;
			assign RD2 = (rt!=0)?rf[rt]:0;
					
endmodule

module imem( input  [5:0] I, output [31:0] RD); 

  reg  [31:0] RAM[63:0];

  initial begin
      $readmemh("memfile.dat",RAM);
  end

  assign RD = RAM[I]; 

endmodule


module dmem(input clk, input memWrite, input [31:0] A, input [31:0] WD, output [31:0] RD);

		reg [31:0] RAM [63:0];

		assign RD = RAM[A[31:2]];
		
		always@(posedge clk) begin
		
			if(memWrite==1'b1) begin
					
				RAM[A[31:2]]<=WD;
				
			end
				
		end

endmodule

//adder

module adder(input [31:0] in0, input [31:0] in1, output [31:0] out);

		assign out = in0+in1;
	
endmodule



module sl2(input [31:0] in, output [31:0] out);

		assign out = {in[29:0],2'b00};

endmodule



module signext(input [15:0] in, output reg [31:0] signExtIn);


   always@(in)
		begin
		signExtIn[15:0]=in;
		case(in[15])
			1'b0: signExtIn[31:16] = 16'b0000000000000000;
			1'b1: signExtIn[31:16] = 16'b1111111111111111;
		endcase
	end
endmodule


module mux2to1_5bits(input [4:0] in0, input [4:0] in1, input select, output reg [4:0] muxOut);
    
	always@(in0,in1,select)
	begin
	case(select)
		1'b0: muxOut=in0;
		1'b1: muxOut=in1;
	endcase
	end
endmodule

module mux2to1_32bits(input [31:0] in0, input [31:0] in1, input select, output reg [31:0] muxOut);
      
	always@(in0,in1,select)
	begin
	case(select)
		1'b0: muxOut=in0;
		1'b1: muxOut=in1;
	endcase
	end
endmodule

module flopr #(parameter WIDTH = 8)
				  (input clk, input reset,
					input      [WIDTH-1 : 0] d,
					output reg [WIDTH-1 : 0] q);
					
		always@(posedge clk, posedge reset) begin
			
			if(reset) q<=0;
			else		 q<=d;
			
		end

endmodule


//alu


module alu(	input [31:0] in0, in1, 
            input [2:0] select, 
				output reg [31:0] out, output Zero);
				
	always @ ( * )
		case (select)
			3'b000: out <= in0 & in1;
			3'b001: out <= in0 | in1;
			3'b010: out <= in0 + in1;
			3'b011: out <= in0 & ~in1;
			3'b101: out <= in0 + ~in1;
			3'b110: out <= in0 - in1;
			3'b111: out <= in0 < in1 ? 1:0;
			default: out <= 0; 
		endcase
	
	assign Zero = (out == 32'b0);
	
endmodule

// control circuit

module controller(input [5:0] op, input [5:0] funct, input zero,
						output memToReg, output memWrite,
						output pcSrc, output aluSrc,
						output regDst, output regWrite,
						output jump,
						output [2:0] aluControl);

		wire [1:0] aluOp;
		wire branch;
		
		mainDecoder md(op,memToReg,memWrite,branch,aluSrc,regDst,regWrite,jump,aluOp);
		aluDecoder ad(funct,aluOp,aluControl);
		
		assign pcSrc = branch & zero;
		
		
endmodule

module mainDecoder(input [5:0]op,
				   output memToReg, output memWrite,
					output branch, output  aluSrc,
					output regDst, output regWrite,
					output jump,
					output [1:0] aluOp);
					

		reg [8:0] controls;
		assign {regWrite,regDst,aluSrc,branch,memWrite,memToReg,jump,aluOp} = controls;
		
		always@(*) begin
			
			case(op)
				6'b000000 : controls <= 9'b110000010;//Rtype
				6'b100011 : controls <= 9'b101001000;//LW
				6'b101011 : controls <= 9'b001010000;//SW
				6'b000100 : controls <= 9'b000100001;//BEQ
				6'b001000 : controls <= 9'b101000000;//ADDI
				6'b000010 : controls <= 9'b000000100;//J
				default   : controls <= 9'bxxxxxxxxx;//illegal
			endcase
			
		end
		

endmodule


module aluDecoder(input [5:0]funct, input [1:0]aluOp, output  reg [2:0] aluControl);
	
	always@(*) begin
		
		case(aluOp)
			2'b00 : aluControl <= 3'b010; //add for lw sw addi
			2'b01 : aluControl <= 3'b110; //sub for beq
			default: 
					   case(funct) //R type
							6'b100000 : aluControl <= 3'b010; //add
							6'b100010 : aluControl <= 3'b110; //sub
							6'b100100 : aluControl <= 3'b000; //and
							6'b100101 : aluControl <= 3'b001; //or
							6'b101010 : aluControl <= 3'b111; //slt
							default   : aluControl <= 3'bxxx; //illegal
						endcase
		endcase
		
	end


endmodule


// Datapath

module datapath(input  clk, reset,
				input  memToReg, pcSrc,
				input  aluSrc, regDst,
				input  regWrite, jump,
				input  [2:0] aluControl,
				output zero,
				output [31:0] pc,
				input  [31:0] instr,
				output [31:0] aluout, writeData,
				input  [31:0] readData);
				
		wire [4:0] writereg;
		wire [31:0] pcNext, pcNextbr, pcPlus4, pcBranch;
		wire [31:0] signImm, signImmsh;
		wire [31:0] src1, src2;
		wire [31:0] result;
		
		// next PC logic
		flopr #(32) pcreg(clk, reset, pcNext, pc);
		adder pcadd1(pc, 32'b100, pcPlus4);
		sl2 immsh(signImm, signImmsh);
		adder pcadd2(pcPlus4, signImmsh, pcBranch);
		mux2to1_32bits pcbrmux(pcPlus4, pcBranch, pcSrc, pcNextbr);
		mux2to1_32bits pcmux(pcNextbr, {pcPlus4[31:28], instr[25:0], 2'b00}, jump, pcNext);
		
		// register file logic
		regfile rf(clk, regWrite, instr[25:21], instr[20:16], writereg, result, src1, writeData);
		mux2to1_5bits wrmux(instr[20:16], instr[15:11], regDst, writereg);
		mux2to1_32bits resmux(aluout, readData, memToReg, result);
		signext se(instr[15:0], signImm);
		
		// ALU logic
		mux2to1_32bits src2mux(writeData, signImm, aluSrc, src2);
		alu alu(src1, src2, aluControl, aluout, zero);


endmodule



module mips(input  clk, reset,
				output [31:0] pc,
				input  [31:0] instr,
				output memWrite,
				output [31:0] aluout, writeData,
				input  [31:0] readData);

			wire memToReg, aluSrc, regDst, regWrite, jump, pcSrc, zero;
			wire [2:0] aluControl;
				
			controller c(instr[31:26], instr[5:0], zero, memToReg, memWrite, 
						 pcSrc, aluSrc, regDst, regWrite, jump, aluControl);
			
			datapath dp(clk, reset, memToReg, pcSrc, aluSrc, regDst, regWrite,
             			jump, aluControl, zero, pc, instr, aluout, writeData, readData);

endmodule



//top module

module top(input  clk, reset,
		   output [31:0] writeData, dataAdr,
		   output memWrite);

	wire [31:0] pc, instr, readData;
	
	// instantiate processor and memories
	
	mips mips(clk, reset, pc, instr, memWrite, dataAdr,writeData, readData);
	
	imem imem(pc[7:2], instr);
	
	dmem dmem(clk, memWrite, dataAdr, writeData, readData);

endmodule



//test bench

module testbench();

	reg clk;
	reg reset;
	wire [31:0] writeData, dataAdr;
	wire memWrite;
	
	// instantiate device to be tested
	top dut (clk, reset, writeData, dataAdr, memWrite);
	
	// initialize test
	initial
	begin
		reset <= 1; # 22; reset <= 0;
	end
	
	// generate clock to sequence tests
	always
	begin
		clk <= 1; # 5; clk <= 0; # 5;
	end
	
	// check results
	always @(negedge clk)
	begin
		if (memWrite) begin
			if (dataAdr===84 & writeData===7) begin
				$display("Simulation succeeded");
				$stop;
				end else if (dataAdr !==80) begin
				$display("Simulation failed");
				$stop;
			end
		end
	end

endmodule
