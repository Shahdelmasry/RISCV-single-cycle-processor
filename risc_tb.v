// Testbench for RSICV module

 module risc_tb();
    reg clk;
    reg reset;
    wire [31:0] WriteData, DataAdr;
    wire MemWrite;
   
  //  wire [31:0] instruction; // Output from instruction memory

    
    // Instantiate device under test
    top dut(
        .clk(clk),
        .reset(reset),
        .write_data(WriteData),
        .DataAdr(DataAdr),
        .MemWrite(MemWrite)
    );
    // Initialize test
    initial begin
        reset <= 1;   
        #22; 
        reset <= 0;
        $dumpfile("waveform.vcd");
        $dumpvars(0, risc_tb);
        $dumpvars(0, dut); // Dump entire DUT
      //  $display("program counter = %h", pc);
      //  #10; // Wait for reset to complete      
     
    end
    
    // Generate clock (10ns period)
    always begin
        clk <= 1; #5; 
        clk <= 0; #5;
    end
   

     
  
    // Check results on negative clock edge
    always @(negedge clk) begin
        if(MemWrite) begin
            if(DataAdr === 100 && WriteData === 25) begin
                $display("Simulation succeeded at time %0t", $time);
                $finish;  // Use $finish instead of $stop for proper termination
            end 
            else if (DataAdr !== 96) begin
                $display("Simulation failed at time %0t", $time);
                $display("DataAdr = %0d, WriteData = %0d", DataAdr, WriteData);
                $stop;  // Stop simulation on failure
            end
        end
    end
initial begin 
    #1000;
    $display("Simulation timed out");
    $stop;
end
    
    
    
endmodule