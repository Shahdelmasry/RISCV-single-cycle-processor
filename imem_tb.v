module imem_tb();

    wire [31:0] instruction; // Output from instruction memory
    reg [31:0] pc; // Program counter output

    // Instantiate device under test
    instruction_memory imemtb(
        .pc(pc),
        .instruction(instruction)
    );

    // Initialize test
    initial begin
       
        $dumpfile("waveform.vcd");
        $dumpvars(0, imem_tb);
        $dumpvars(0, imemtb.RAM); // Dump memory array
        $dumpvars(0, imemtb.instruction); // Dump output
       
        pc <= 32'h00000000;
        #10; 
        pc <= 32'h00000004; 
        #10;
        pc <= 32'h00000008;
        #10;
        pc <= 32'h0000000C;

    end

   
endmodule