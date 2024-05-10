module MultiCycleProcessor64BitInMIPS(input CLK, RST,
												  output [63:0] WD, ADDR,
												  output MemWrite);
		wire [63:0] RD;
		mips mips(CLK, RST, ADDR, WD, MemWrite, RD);
		mem memory(CLK, MemWrite, ADDR, WD, RD);
endmodule

module datapath(input CLK, RST, 
					 input PCEn, IRWrite, RegWrite, 
					 input ALUSrcA, IorD, MemToReg, RegDst, 
					 input[1:0] ALUSrcB, 
					 input PCSrc, 
					 input [2:0] ALUControl, 
					 output [5:0] OP, Funct, 
					 output Zero, 
					 output [63:0] ADDR, WD,
					 input [63:0] RD);
		
		wire [4:0] WriteReg;
		wire [63:0] PCNext, PC;
		wire [63:0] Instr, Data, SrcA, SrcB;
		wire [63:0] A;
		wire [63:0] ALUResult, ALUOut;
		wire [63:0] SignImm;
		wire [63:0] SignImmsh;
		wire [63:0] WD3, RD1, RD2;
		
		assign OP = Instr[31:26];
		assign Funct = Instr[5:0];
		
		flopenr #(64) PCReg(CLK, RST, PCEn, PCNext, PC);
		mux2 #(64) ADRMux(PC, ALUOut, IorD, ADDR);
		flopenr #(64) InstrReg(CLK, RST, IRWrite, RD, Instr);
		flopr #(64)DataReg(CLK, RST, RD, Data);
		
		mux2 #(5) RegDstMux(Instr[20:16], Instr[15:11], RegDst, WriteReg);
		// Instr[20:16] corresponds to target register
		// Instr [15:11] corresponds to source register
		mux2 #(64) WDMux(ALUOut, Data, MemToReg, WD3);
		regfile register_fie(CLK, RegWrite, Instr[25:21], Instr[20:16], WriteReg, WD3, RD1, RD2);
		signext se(Instr[15:0], SignImm);
		sl2 immsh2bits(SignImm, SignImmsh);
		flopr #(64) AReg(CLK, RST, RD1, A);
		flopr #(64) BReg(CLK, RST, RD2, WD);
		mux2 #(64) SrcAMux(PC, A, ALUSrcA, SrcA);
		mux4 #(64) SrcBMux(WD, 64'b100, SignImm, SignImmsh, ALUSrcB, SrcB);
		ALU alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
		flopr #(64) ALUReg(CLK, RST, ALUResult, ALUOut);
		mux2 #(64) PCMux(ALUResult, ALUOut, PCSrc, PCNext);
endmodule

module ALU(input [63:0] IN1, IN2, input[2:0] CNTRL, output reg [63:0] OUT, output Zero);
	always @(*)
		case(CNTRL[2:0])
			3'b000: OUT <= IN1&IN2; // AND
			3'b001: OUT <= IN1|IN2; // OR
			3'b010: OUT <= IN1+IN2; // ADD
			3'b110: OUT <= IN1-IN2; // SUB
			3'b111: OUT <= IN1<IN2; // SLT
			default OUT <= 0; // default to zero
		endcase
		assign Zero = (OUT==64'b0);
endmodule

module sl2(input [63:0] A, output [63:0] O);
	assign O = {A[61:0], 2'b00};
endmodule

module flopenr #(parameter WIDTH = 8)
					 (input CLK, RST,
					  input EN,
					  input [WIDTH-1:0] D,
					  output reg [WIDTH-1:0] Q);
		always@(posedge CLK, posedge RST)
				if(RST) Q <= 0;
				else if(EN) Q <= D; 
endmodule


module flopr #(parameter WIDTH = 8)
				  (input CLK, RST,
					input [WIDTH-1:0] D,
					output reg [WIDTH-1:0] Q);
		always@(posedge CLK, posedge RST)
				if (RST) Q <= 0;
				else		Q <= D;
endmodule

module signext (input [15:0] A, output [63:0] Y);
			assign Y = {{48{A[15]}},A};
endmodule

module mux2 #(parameter WIDTH = 8)
					(input [WIDTH-1:0] D0, D1,
					 input S,
					 output [WIDTH-1:0] Y);
					 assign Y = S ? D1 :D0;
endmodule

module mux4 #(parameter WIDTH = 8)
				(input [WIDTH-1:0] D0, D1, D2, D3,
				 input [1:0] S,
				 output reg [WIDTH-1:0]Y);
			always @(*)
					case(S)
							2'b00: Y <= D0;
							2'b01: Y <= D1;
							2'b10: Y <= D2;
							2'b11: Y <= D3;
					endcase
endmodule

module mem(input CLK, WE, input [63:0] A, WD, output [63:0] RD);
		reg [63:0] RAM[255:0];
		initial 
				begin
					$readmemh("/home/rajesh/intelFPGA_lite/20.1/quartus/bin/memfile.txt", RAM);
				end
		assign RD = RAM[A[63:2]];
		
		always @(posedge CLK)
			if(WE) begin
				RAM[A[63:2]] <= WD;
				$writememh("/home/rajesh/intelFPGA_lite/20.1/quartus/bin/memfile.txt", RAM);
			end
endmodule

module controller(input CLK, RST,
						input [5:0] OP, Funct,
						input Zero,
						output PCEn, MemWrite, IRWrite, RegWrite,
						output ALUSrcA, IorD, MemtoReg, RegDst,
						output [1:0] ALUSrcB,
						output PCSrc,
						output [2:0] ALUControl);
		wire [1:0] ALUOp;
		wire BRANCH, PCWrite;
		maindec md(CLK, RST, OP,
					  PCWrite, MemWrite, IRWrite, RegWrite,
					  ALUSrcA, BRANCH, IorD, MemtoReg, RegDst,
					  ALUSrcB, PCSrc, ALUOp);
		aludec ad(Funct, ALUOp, ALUControl);
		assign PCEn = PCWrite | (BRANCH & Zero);
endmodule


module maindec(input CLK, RST,
					input [5:0] OP,
					output PCWrite, MemWrite, IRWrite, RegWrite,
					output ALUSrcA, Branch, IorD, MemtoReg, RegDst,
					output [1:0] ALUSrcB,
					output PCSrc,
					output [1:0] ALUOp);
		
		//FSM states
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
		//MIPS Instruction Opcodes
		parameter LW = 6'b100011;
		parameter SW = 6'b101011;
		parameter RTYPE = 6'b000000;
		parameter BEQ = 6'b000100;
		parameter ADDI = 6'b001000;
		
		
		reg [4:0] state, nextstate;
		reg [16:0] controls;
		
		// state register
		always @(posedge CLK or posedge RST)
			if(RST) state <= FETCH;
			else state <= nextstate;
		
		always@(*)
			case(state)
					FETCH: nextstate <= DECODE;
					DECODE: case(OP)
							LW: nextstate <= MEMADR;
							SW: nextstate <= MEMADR;
							RTYPE: nextstate <= EXECUTE;
							BEQ: nextstate <= BRANCH;
							ADDI: nextstate <= ADDIEXECUTE;
							default: nextstate <= FETCH;
					endcase
					MEMADR: case(OP)
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
					default: nextstate <= FETCH;
			endcase
			
			assign {PCWrite, MemWrite, IRWrite, RegWrite,
					  ALUSrcA, Branch, IorD, MemtoReg, RegDst,
					  ALUSrcB, PCSrc, ALUOp} = controls;
					  
			always @(*)
				case(state)
						FETCH: controls <= 16'b1010_00000_01_0_00;
						DECODE: controls <= 16'b0000_00000_11_0_00;
						MEMADR: controls <= 16'b0000_00000_10_0_00;
						MEMRD: controls <= 16'b0000_00100_00_0_00;
						MEMWB: controls <= 16'b0001_00010_00_0_00;
						MEMWR: controls <= 16'b0100_00100_00_0_00;
						EXECUTE: controls <= 16'b0000_10000_00_0_10;
						ALUWRITEBACK: controls <= 16'b0001_00001_00_0_00;
						BRANCH: controls <= 16'b0000_11000_00_1_01;
						ADDIEXECUTE: controls <= 16'b0000_100000_10_0_00;
						ADDIWRITEBACK: controls <= 16'b0001_00000_00_0_00;
						default: controls <= 16'b0000_xxxxx_xxxx_xx;
				endcase
endmodule

module aludec(input [5:0] Funct, input [1:0] ALUOp, output reg [2:0] ALUControl);
			always @(*)
				case (ALUOp)
					2'b00: ALUControl <= 3'b010;
					2'b01: ALUControl <= 3'b110;
					default: case(Funct)
							6'b100000: ALUControl <= 3'b010;
							6'b100010: ALUControl <= 3'b110;
							6'b100100: ALUControl <= 3'b000;
							6'b100101: ALUControl <= 3'b001;
							6'b101010: ALUControl <= 3'b111;
							default: ALUControl <= 3'bxxx;
					endcase
				endcase
endmodule

module regfile(input CLK,
					input WE3,
					input [4:0] RA1, RA2, WA3,
					input [63:0] WD3,
					output[63:0] RD1, RD2);
					
		reg [63:0] register_file[31:0];
		
		always @(posedge CLK)
			if(WE3) register_file[WA3] <= WD3;	
			
			assign RD1=(RA1!= 0)?register_file[RA1]:0;
			assign RD2=(RA2!= 0)?register_file[RA2]:0;		
endmodule


module mips(input CLK, RST,
				output [63:0] ADDR, WD,
				output MemWrite,
				input [63:0] RD);
		 
		 wire Zero, PCEn, IRWrite, RegWrite, ALUSrcA, IorD, MemtoReg, RegDst;
		 wire [1:0] ALUSrcB;
		 wire PCSrc;
		 wire [2:0] ALUControl;
		 wire [5:0] OP, Funct;
		 
		 controller c(CLK, RST, OP, Funct, Zero,
						  PCEn, MemWrite, IRWrite, RegWrite, 
						  ALUSrcA, IorD, MemtoReg, RegDst,
						  ALUSrcB, PCSrc, ALUControl);
		 datapath dp(CLK, RST,
						 PCEn, IRWrite, RegWrite,
						 ALUSrcA, IorD, MemtoReg, RegDst,
						 ALUSrcB, PCSrc, ALUControl,
						 OP, Funct, Zero,
						 ADDR, WD, RD);
endmodule
