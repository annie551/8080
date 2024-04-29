`timescale 1ps/1ps

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
