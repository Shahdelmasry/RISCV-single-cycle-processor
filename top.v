module top(input clk,          // System clock
    output [31:0] DataAdr,     // aluresult
    output [31:0] write_data, // Data to write to memory
    output MemWrite,      // Memory write enable signal
    input reset     );

    wire [31:0] pc, instr,  read_data; // Data wires
RISCV risc (
        .clk(clk),          // System clock
        .instr(instr),     // Instruction input
        .write_data(write_data), // Data to write to memory
        .mem_write(MemWrite), // Memory write enable signal
        .pc(pc),           // Program counter output
        .ReadData(read_data), // Data read from memory
        .ALUResult(DataAdr), // ALU result output
        .reset(reset)      // Reset signal
    );
instruction_memory imem (
        .pc(pc), // Address input
        .instruction(instr) // Instruction output
    );   
data_memory dmem (
        .clk(clk),
        .address(DataAdr),
        .write_data(write_data),
        .WE(MemWrite),
        .read_data(read_data)
    );
endmodule