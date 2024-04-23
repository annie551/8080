`timescale 1ps/1ps

module regs(input clk,
    input [3:0]raddr0_, output [15:0]rdata0,
    input [3:0]raddr1_, output [15:0]rdata1,
    input wen, input [3:0]waddr0, input [15:0]wdata0,
    input [3:0] waddr1, input [15:0] wdata1);

    reg [15:0]data[0:7];

    reg [3:0]raddr0;
    reg [3:0]raddr1;

    assign rdata0 = data[raddr0];
    assign rdata1 = data[raddr1];

    always @(posedge clk) begin
        raddr0 <= raddr0_;
        raddr1 <= raddr1_;
        if (wen) begin
            data[waddr0] <= wdata0;
            data[waddr1]<=wdata1;
        end
    end

endmodule
