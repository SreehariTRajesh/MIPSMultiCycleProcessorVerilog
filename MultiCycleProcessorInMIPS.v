module MultiCycleImplementationOfMIPS(input clk, reset, output [63:0] writedata, addr, output memwrite);
	wire [63:0] readdata;
	mips mips(clk, reset, addr, writedata, memwrite, readdata);
	mem mem(clk, memwrite, addr, writedata, readdata);
endmodule


module ALU( input [63:0] A, B, input [2:0] ALUControl, output reg [63:0] Y, output Zero);
		always @(*)
			case (ALUControl[2:0])
				3'b000: Y <= A&B; // AND
				3'b001: Y <= A|B; // OR
				3'b010: Y <= A+B; // ADD
				3'b110: Y <= A-B; // SUB
				3'b111: Y <= 0;
			endcase
		assign Zero = (Y==32'b0);
endmodule
 

module DataPath(input clk, reset, 
                input pcen, IRWrite, RegWrite, 
                input ALUSrcA, IorD, MemToReg, RegDst,
                input [1:0] ALUSrcB, PCSrc,
                input [2:0] AluControl, 
                output [5:0] op, funct,
                output zero,
                output [63:0] addr, writedata,
                input [63:0] readdata);
					 
					 wire [4:0] writereg;
					 wire [63:0] pcnext, pc;
					 wire [63:0] instr;
					 wire [63:0] data, srca, srcb;
					 wire [63:0] a;
					 wire [63:0] aluresult, aluout;
					 wire [63:0] signimm;
					 wire [63:0] signimmsh;
					 wire [63:0] wd3, rd1, rd2;
					 
					 assign op = instr[31:26];
					 assign funct = instr[5:0];
					 
					 flopenr #(64) pcreg(clk, reset, pcen, pcnext, pc);
					 mux2 #(64) adrmux(pc, aluout, IorD, addr);
					 flopenr #(64) instrreg(clk, reset, IRWrite, readdata, instr);
					 flopr #(64) datareg(clk, reset, readdata, data);
					 
					 mux2 #(5) regdstmux(instr[20:16], instr[15:11], RegDst, writereg);
					 mux2 #(64) wdmux(aluout, data, MemToReg, wd3);
					 regfile rf(clk, RegWrite, instr[25:21], instr[20:16], writereg, wd3, rd1, rd2);
					 signext se(instr[15:0], signimm);
					 sl2 immsh(signimm, signimmsh);
					 
					 flopr #(64) areg(clk, reset, rd1, a);
					 flopr #(64) breg(clk, reset, rd2, writedata);
					 mux2 #(64) srcamux(pc, a, ALUSrcA, srca);
					 mux4 #(64) srcbmux(writedata, 32'b100, signimm, signimmsh, ALUSrcB, srcb);
					 
					 ALU alu(srca, srcb, AluControl, aluresult, zero);
					 
					 flopr #(64) alureg(clk, reset, aluresult, aluout);
					 mux3 #(64) pcmux(aluresult, aluout, {pc[31:28], instr[25:0], 2'b00}, PCSrc, pcnext);
endmodule


module flopenr #(parameter WIDTH=8)
					 (input clk, reset, 
					  input en,
					  input [WIDTH-1:0] d,
					  output reg [WIDTH-1:0] q);
		always@(posedge clk, posedge reset)
				if(reset) q <= 0;
				else if (en) q <= d;
endmodule


module flopr #(parameter WIDTH=8)
					(input clk, reset,
					 input [WIDTH-1:0] d,
					 output reg[WIDTH-1:0] q);
		 always@(posedge clk, posedge reset)
				if(reset) q <= 0;
				else 	q <= d;
endmodule


module signext(input [15:0] a, output [63:0] y);
		assign y = {{64{a[15]}}, a};
endmodule

module sl2(input [63:0] a, output [63:0] y);
	assign y = {a[61:0], 2'b00};
endmodule

module mux2 #(parameter WIDTH = 8)
			(input [WIDTH-1:0] d0, d1,
			 input s,
			 output [WIDTH-1:0] y);
			 assign y = s ? d1: d0;
endmodule

module mux3 #(parameter WIDTH = 8)
				(input [WIDTH-1:0] d0, d1, d2,
				 input [1:0] s,
				 output [WIDTH-1:0] y);
				assign #1 y = s[1] ? d2: (s[0] ? d1: d0);
endmodule

module mux4 #(parameter WIDTH = 8)
				(input [WIDTH-1:0] d0, d1, d2, d3,
				 input [1:0] s,
				 output reg [WIDTH-1:0] y);
			always @(*)
				case (s)
						2'b00: y <= d0;
						2'b01: y <= d1;
						2'b10: y <= d2;
						2'b11: y <= d3;
				endcase
endmodule


module mem(input clk, we, input [63:0] a, wd, output [63:0] rd);
	reg [63:0] RAM[63:0];
	assign rd = RAM[a[63:2]];
	initial 
		begin
				RAM[0] <= 32'h200201c2;
				RAM[1] <= 32'h20030226;
				RAM[2] <= 32'h20030226;
				RAM[3] <= 32'h00432020;
				RAM[4] <= 32'hac040014;
		end
	always @(posedge clk)
			if(we)
			RAM[a[63:2]] <= wd;
endmodule

module controller(input clk, reset, input [5:0] op, funct, input zero,
						output pcen, memwrite, IRWrite, RegWrite, 
						output ALUSrcA, IorD, MemToReg, RegDst,
						output [1:0] ALUSrcB,
						output [1:0] PCSrc,
						output [2:0] AluControl);
			wire [1:0] aluop;
			wire branch, pcwrite;
			
			maindec md(clk, reset, op, 
						  pcwrite, memwrite, IRWrite, RegWrite,
						  ALUSrcA, branch, IorD, MemToReg, RegDst,
						  ALUSrcB, PCSrc, aluop);
			aludec(funct, aluop, AluControl);
			assign pcen = pcwrite | branch & zero;
endmodule


module maindec(input clk, reset,
					input [5:0] op,
					output PCWrite, MemWrite, IRWrite, RegWrite, 
					output ALUSrcA, Branch, IorD, MemToReg, RegDst,
					output [1:0] ALUSrcB,
					output [1:0] PCSrc,
					output [1:0] ALUOp);
		
		parameter FETCH = 5'b00000;
		parameter DECODE = 5'b00001;
		parameter MEMADR = 5'b00010;
		parameter MEMRD = 5'b00011;
		parameter MEMWB = 5'b00100;
		parameter MEMWR = 5'b00101;
		parameter EXECUTE = 5'b00110;
		parameter ALUWRITEBACK = 5'b00111;
		parameter BRANCH = 5'b01000;
		parameter ADDIEXECUTE = 5'b01001;
		parameter ADDIWRITEBACK = 5'b01010;
		parameter JUMP = 5'b01011;
		
		parameter LW = 6'b100011;
		parameter SW = 6'b101011;
		parameter RTYPE = 6'b000000;
		parameter BEQ = 6'b000100;
		parameter ADDI = 6'b001000;
		parameter J = 6'b000010;
		
		reg [4:0] state, nextstate;
		reg [16:0] controls;
		
		always @(posedge clk or posedge reset)
			if (reset) state <= FETCH;
			else state <= nextstate;
		
		always @(*)
			case (state)
					FETCH: nextstate <= DECODE;
					DECODE: case(op)
							LW: nextstate <= MEMADR;
							SW: nextstate <= MEMADR;
							RTYPE: nextstate <= EXECUTE;
							BEQ: nextstate <= BRANCH;
							ADDI: nextstate <= ADDIEXECUTE;
							J: nextstate <= JUMP;
							default: nextstate <= FETCH;
					endcase
					MEMADR: case(op)
									LW: nextstate <= MEMRD;
									SW: nextstate <= MEMWR;
									default: nextstate <= FETCH;
							endcase
					MEMRD: nextstate <= MEMWB;
					MEMWB: nextstate <= FETCH;
					MEMWR: nextstate <= FETCH;
					EXECUTE: nextstate <= ALUWRITEBACK;
					ALUWRITEBACK: nextstate <= FETCH;
					BRANCH: nextstate <= FETCH;
					ADDIEXECUTE: nextstate <= ADDIWRITEBACK;
					ADDIWRITEBACK: nextstate <= FETCH;
					JUMP: nextstate <= FETCH;
					default: nextstate <= FETCH;
			endcase
		assign {PCWrite, memwrite, IRWrite, RegWrite, ALUSrcA, Branch, IorD, MemToReg, RegDst, ALUSrcB, PCSrc, ALUOp} = controls;
		
		always @(*)
			case (state)
				FETCH: controls <= 19'b1010_00000_0100_00;
				DECODE: controls <= 19'b0000_00000_1100_00;
				MEMADR: controls <= 19'b0000_10000_1000_00;
				MEMRD: controls <= 19'b0000_00100_0000_00;
				MEMWB: controls <= 19'b0001_00010_0000_00;
				MEMWR: controls <= 19'b0100_00100_0000_00;
				EXECUTE: controls <= 19'b0000_10000_0000_10;
				ALUWRITEBACK: controls <= 19'b0001_00001_0000_00;
				BRANCH: controls <= 19'b0000_11000_0001_01;
				ADDIEXECUTE: controls <= 19'b0000_10000_1000_00;
				ADDIWRITEBACK: controls <= 19'b0001_00000_0000_00;
				JUMP: controls <= 19'b1000_00000_0010_00;
				default: controls <= 19'b0000_xxxxx_xxxx_xx;
			endcase
endmodule
		
module aludec(input [5:0] funct, input [1:0] aluop, output reg [2:0] alucontrol);
		always@(*)
			case(aluop)
				3'b000: alucontrol <= 3'b010;
				3'b001: alucontrol <= 3'b010;
				3'b010: case(funct)
					6'b100000: alucontrol <= 3'b010;
					6'b100010: alucontrol <= 3'b110;
					6'b100100: alucontrol <= 3'b000;
					6'b100101: alucontrol <= 3'b001;
					6'b101010: alucontrol <= 3'b111;
					default: alucontrol <= 3'bxxx;
				endcase
				default: alucontrol <= 3'bxxx;
			endcase
endmodule

module regfile(input clk, input we3, input [4:0] ra1, ra2, wa3, input [31:0] wd3, output [31:0] rd1, rd2);
		reg [63:0] rf[31:0];
		
		always @(posedge clk)
				if(we3) rf[wa3] <= wd3;
				assign rd1 = (ra1 !=0) ? rf[ra1] : 0;
				assign rd2 = (ra2 !=0) ? rf[ra2] : 0;
endmodule

module mips(input clk, reset, output [63:0] addr, writedata, output memwrite, input [63:0] readdata);
		wire zero, pcen, irwrite, regwrite, iord, memtoreg, regdst, ALUSrcA;
		wire [1:0] ALUSrcB;
		wire [1:0] PCSrc;
		wire [2:0] ALUControl;
		wire [5:0] op, funct;
		controller c (clk, reset, op, funct, zero, pcen, memwrite, irwrite, regwrite, ALUSrcA, iord, memtoreg, regdst, ALUSrcB, PCSrc, ALUControl);
		DataPath dp(clk, reset, pcen, irwrite, regwrite, ALUSrcA, iord, memtoreg, regdst, ALUSrcB, PCSrc, ALUControl, op, funct, zero, addr, writedata, readdata);
endmodule


module testbenchv1;
	reg clk;
	reg reset;
	wire [31:0] writedata, dataaddr;
	wire memwrite;
	
	MultiCycleImplementationOfMIPS mcycle(clk, reset, writedata, dataaddr, memwrite);
	initial 
	begin
		 reset <= 1; #22; reset <= 0;
	end
	
	always
	begin
		clk <= 1; #5 ; clk <= 0; #5;
	end
	
	always@(negedge clk)
		begin
				if (memwrite) begin
						if(dataaddr == 20 & writedata === 1000) begin
							$display("Simulation Succeeded");
							$stop;
						end else if (dataaddr !== 80) begin
							$display("Failed %h %h", writedata, dataaddr);
							$stop;
						end
				end
		end
endmodule

