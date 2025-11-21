module data_memory (
    input clk,        // System clock
    input [31:0] address, // Address input
    input [31:0] write_data, // Data to write to memory
    input WE,    // Memory write enable signal
    output [31:0] read_data // Data read from memory
 );
    reg [31:0] mem [63:0]; // Memory array of 256 words (32 bits each)
    always @(posedge clk) begin
        if (WE) begin
            mem[address[31:2]] <= write_data; // Write data to memory at the specified address
        end
    end
   
    assign read_data = mem[address[31:2]]; // Read data from memory at the specified address   
endmodule