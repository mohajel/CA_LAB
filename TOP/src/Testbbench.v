module Testbench ();
    reg clk = 0, rst = 0;
    ARM uut(clk,rst);

    always #5 clk = ~clk;    
    
    initial begin
        rst = 1;
        #33 rst = 0;
        #1000 $stop;
    end
endmodule