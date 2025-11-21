module RISCV(
    input clk,          // System clock
    input [31:0] instr,    // aluresult
    output [31:0] write_data, // Data to write to memory
    output mem_write,   // Memory write enable signal
    output [31:0] pc, // Program counter output
    input [31:0] ReadData, // Data read from memory
    output [31:0] ALUResult, // ALU result output
    input reset          
 );
    wire zero_flag,RegWrite,pcsrc,alusrc; // Control signals
    wire [2:0] alu_control; // ALU control signal
    wire[1:0] immsrc; // Immediate source control signal
    wire [1:0] resultsrc; // Result source control signal

    // Instantiate modules
    
    controller cu (
        .instruction(instr), 
        .zero_flag(zero_flag), 
        .alu_control(alu_control), 
        .RegWrite(RegWrite), 
        .mem_write(mem_write), 
        .resultsrc(resultsrc), 
        .immsrc(immsrc), 
        .pcsrc(pcsrc), 
        .alusrc(alusrc)
    );
   
    datapath dp (
        .clk(clk), // System clock
        .reset(reset), // Reset signal
        .ResultSrc(resultsrc), // Result source control signal
        .pcsrc(pcsrc), // Program counter source control signal
        .ALUSrc(alusrc), // ALU source control signal
        .RegWrite(RegWrite), // Register write control signal
        .immsrc(immsrc), // Immediate source control signal
        .ALUControl(alu_control), // ALU control signal
        .zero_flag(zero_flag), // Zero flag output
        .PC(pc), // Program counter output
        .instr(instr), // Instruction output
        .ALUResult(ALUResult), // ALU result output
        .WriteData(write_data), // Write data output
        .ReadData(ReadData) // Data read from memory
    );
endmodule
module datapath(
    input clk, reset, // System clock and reset signal
    input [1:0] ResultSrc, // Result source control signal
    input pcsrc, // Program counter source control signal
    input ALUSrc, // ALU source control signal
    input RegWrite, // Register write control signal
    input [1:0] immsrc, // Immediate source control signal
    input [2:0] ALUControl, // ALU control signal
    output  zero_flag, // Zero flag output
    output [31:0] PC, // Program counter output
    input [31:0] instr, // Instruction output
    output [31:0] ALUResult, // ALU result output
    output [31:0] WriteData, // Write data output
    input [31:0] ReadData // Data read from memory
 );
    wire [31:0] srcA; // Source operands for ALU
    wire [31:0] immExt; // Extended immediate value
    reg [31:0] pc_next, srcB,result;

    program_counter pc_inst (
        .clk(clk), // System clock
        .pc_next(pc_next), // Next program counter value
        .pc(PC), // Current program counter value
        .reset(reset) // Reset signal
    );
    immediate_extender imm_ext (
        .instr(instr),
        .immsrc(immsrc),
        .immExt(immExt)
    );
    alu alu_inst (
        .srcA(srcA),
        .srcB(srcB),
        .alu_control(ALUControl),
        .ALUResult(ALUResult),
        .reset(reset), 
        .zero_flag(zero_flag)
    );
    register_file rf (
        .clk(clk),
        .reset(reset), // Reset signal 
        .rs1(instr[19:15]), 
        .rs2(instr[24:20]), 
        .rd(instr[11:7]), 
        .write_data(result), 
        .write_en(RegWrite), 
        .RD1(srcA), // base address
        .RD2(WriteData)
    );
    always@(*) begin
     case (ResultSrc)
        2'b00: result = ALUResult; 
        2'b01: result = ReadData;
        2'b10: result = PC + 4; 
        default: result = 32'h00000000; // Default case
        endcase
     srcB = ALUSrc ? immExt : WriteData; // Select source B for ALU 
    end
    always @(posedge clk) begin
     if (reset) begin
        pc_next <= 32'h00000000; // Reset program counter to zero
     end
     else begin
     pc_next <= PC  + (pcsrc ? immExt : 4); // Update program counter based on branch condition
     end
    end 
endmodule
module program_counter(
    input clk,reset, // System clock
    input [31:0]pc_next, // Next program counter value
    output reg[31:0] pc // Current program counter value
 );     
        always @(posedge clk) begin
            if (reset) begin
                pc <= 32'h00000000; // Reset program counter to zero
            end
            else
            pc <= pc_next; // Update the program counter on clock edge
            
        end
endmodule
module register_file (
    input clk,  reset,        // System clock
    input [4:0] rs1,    // Source register 1
    input [4:0] rs2,    // Source register 2
    input [4:0] rd,     // Destination register
    input [31:0] write_data, // Data to write to the register file
    input write_en,    // Register write enable signal
    output reg [31:0] RD1, // Data read from rs1
    output reg [31:0] RD2  // Data read from rs2
 ); 
    reg [31:0] registers [31:0]; // 32 registers of 32 bits each
    integer i;
    always @(posedge clk) begin
    if (reset) begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'h00000000; // Initialize all registers to zero on reset
        end
    end
    else begin
        if (write_en) begin
            registers[rd] <= write_data; // Write data to the specified register
        end
    end
    end
    always @(*) begin
        RD1 = registers[rs1]; // Read data from rs1
        RD2 = registers[rs2]; // Read data from rs2
    end    
endmodule
module alu (
    input [31:0] srcA,     // First operand
    input [31:0] srcB,     // Second operand
    input [2:0] alu_control, // ALU control signal
    input reset, // Reset signal
    output reg [31:0] ALUResult, // ALU result
    output reg zero_flag // Zero flag
 );
  
    always @(alu_control,reset,srcB) begin
     if (reset) begin
        ALUResult = 32'h00000000; // Reset result to zero
        zero_flag = 1'b1; // Set zero flag to true
    end
    else begin
        case (alu_control)
            3'b000: ALUResult = srcA + srcB; // ADD
            3'b001: ALUResult = srcA - srcB; // SUB
            3'b010: ALUResult = srcA & srcB; // AND
            3'b011: ALUResult = srcA | srcB; // OR
            3'b100: ALUResult = srcA ^ srcB; // XOR
            3'b101: ALUResult = (srcA < srcB) ? 1 : 0; // SLT
            default: ALUResult = 32'h00000000; // Default case
        endcase
        
        zero_flag = (ALUResult == 32'h00000000); // Set zero flag if result is zero
    end
    end
endmodule
module main_decoder (
    input [6:0] opcode, // Opcode field of the instruction
    output reg [1:0] ALUop, // ALU operation control signal
    output reg RegWrite, // Register write control signal
    output reg mem_write, // Memory write enable signal
    output reg [1:0] resultsrc, // Result source control signal
    output reg [1:0] immsrc, // Immediate source control signal
    output reg branch,jump, 
    output reg alusrc // ALU source control signal
 );
 initial begin
        branch = 0; // Default branch control signal
    end
 always @(*) begin
        casex (opcode) // Opcode field of the instruction
            7'b0110011: begin // R-type instructions
                RegWrite = 1;
                resultsrc =2'b00;  
                branch = 0; 
                mem_write = 0;
                alusrc = 0; 
                ALUop = 2'b10 ; 
                jump=0;
            end
            7'b0000011: begin // I-type
                RegWrite = 1;
                immsrc = 2'b00; 
                ALUop = 2'b00; 
                alusrc = 1; 
                resultsrc = 2'b01; 
                mem_write = 0;
                branch = 0; 
                jump = 0; 
            end
            7'b0100011: begin // S-type 
                RegWrite = 0;
                immsrc = 2'b01; 
                alusrc = 1; 
                resultsrc = 2'b00;
                mem_write = 1; 
                branch = 0; 
                ALUop = 2'b00;
                jump = 0; 
               
            end
            7'b1100011: begin // beq
                RegWrite = 0;
                immsrc = 2'b10; 
                alusrc = 0; 
                resultsrc = 2'b00;
                mem_write = 0; 
                branch = 1; 
                ALUop = 2'b01;
                jump = 0;
            end
            7'b0010011: begin // I-type ALU instructions
                RegWrite = 1;
                immsrc = 2'b00; 
                alusrc = 1; 
                resultsrc =2'b00; 
                mem_write = 0; 
                branch = 0; 
                ALUop = 2'b10;
                jump = 0; 
            end
            7'b1101111: begin // JAL instruction
                RegWrite = 1;
                immsrc = 2'b11;  
                resultsrc =2'b10; 
                mem_write = 0; 
                branch = 0; 
                ALUop = 2'b00;
                alusrc = 0; 
                jump = 1; // Set jump control signal for JAL 
            end
            default: begin 
                RegWrite = 1'bx;
                immsrc = 2'bxx;  
                resultsrc =2'bxx; 
                mem_write = 1'bx; 
                branch = 1'bx; 
                ALUop = 2'bxx;
                alusrc = 1'bx; 
                jump = 1'bx; 
            end 
        endcase
    
    end    
endmodule
module ALU_decoder (
    input [2:0] funct3, 
    input  funct7,op,
    input [1:0] ALUop, 
    output reg [2:0] alu_control
 );
    always @(*) begin
        if (ALUop == 2'b10) begin // R-type instructions
            case (funct3)
                3'b000: if ({op, funct7} == 2'b11) begin
                            alu_control = 3'b001; // SUB
                        end else begin
                            alu_control = 3'b000; // ADD
                        end
                3'b111: alu_control = 3'b010; // AND
                3'b110: alu_control = 3'b011; // OR
                3'b010: alu_control = 3'b101; // set less than (SLT)
                default: alu_control = 3'b000; // Default case
            endcase
        end
        else if (ALUop == 2'b00) begin // I-type instructions
             alu_control = 3'b000; // ADD for load/store
        end 
        else if (ALUop == 2'b01) begin // Branch instructions
            alu_control =  3'b001 ; // BEQ or BNE
        end 
        else begin 
            alu_control = 0;
        end
    end
endmodule
module controller (
   
    input [31:0] instruction, // Instruction input
    input zero_flag, // Zero flag from ALU
    output  [2:0] alu_control, // ALU control signal
    output  RegWrite, // Register write control signal
    output  mem_write, // Memory write enable signal
    output [1:0] resultsrc, // Result source control signal
    output  [1:0] immsrc, // Immediate source control signal
    output  pcsrc, // Program counter source control signal
    output  alusrc  // ALU source control signal

 );
    wire branch, jump; 
    wire [1:0] ALUop;
 main_decoder md (
     .opcode(instruction[6:0]),
     .ALUop(ALUop),
     .RegWrite(RegWrite),
     .mem_write(mem_write),
     .resultsrc(resultsrc),
     .immsrc(immsrc),
     .branch(branch),
     .jump(jump),
     .alusrc(alusrc)
 );
 ALU_decoder ad (
     .funct3(instruction[14:12]),
     .funct7(instruction[30]),
     .op(instruction[5]),
     .ALUop(ALUop),
     .alu_control(alu_control)
 );
 assign pcsrc = (branch & zero_flag)|jump ; // Set pcsrc if branch condition is met
endmodule
module immediate_extender (
    input [31:0] instr, // Instruction input
    input [1:0] immsrc, // Immediate source control signal
    output reg [31:0] immExt // Extended immediate value
 );
    always @(*) begin
        case (immsrc)
            2'b00: immExt = {{20{instr[31]}}, instr[31:20]}; // I-type immediate extension
            2'b01: immExt = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type immediate extension
            2'b10: immExt = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type immediate extension
            2'b11: immExt = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type immediate extension
            default: immExt = 32'h00000000; // Default case
        endcase
    end
endmodule

