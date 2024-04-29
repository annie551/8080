`timescale 1ps/1ps


//abstraction of an external device that gives interrupt signals
// this one is just made to interrupt after 11 cycles and gives a restart instruction but
// it can be made more complicated if desired as this is just a simple example
module interrupter(input clk, output interrupt, output[23:0] interrupt_instruction);

    reg[7:0] counter=0;
    reg interrupt_reg;
    assign interrupt=interrupt_reg;
    assign interrupt_instruction=(8'b11111111)*256*256;

    always @(posedge clk) begin
        counter<=counter+1;
        interrupt_reg<=(counter==11);
    end

endmodule
