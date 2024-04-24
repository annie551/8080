`timescale 1ps/1ps

module regs(input clk,
    input [2:0]raddr0_, output [7:0]rdata0,
    input [2:0]raddr1_, output [7:0]rdata1,
    input [2:0]raddr2_, output [7:0]rdata2,
    input [2:0]raddr3_, output [7:0]rdata3,
    input wen0, input wen1, input wen2, input wen3, 
    input [2:0]waddr0, input [7:0]wdata0,
    input [2:0] waddr1, input [7:0] wdata1,
    input [2:0]waddr2, input [7:0]wdata2,
    input [2:0] waddr3, input [7:0] wdata3);

    reg [7:0]data[0:7];

    reg [3:0]raddr0;
    reg [3:0]raddr1;
    reg [3:0]raddr2;
    reg [3:0]raddr3;

    assign rdata0 = data[raddr0];
    assign rdata1 = data[raddr1];
    assign rdata2 = data[raddr2];
    assign rdata3 = data[raddr3];

    always @(posedge clk) begin
        raddr0 <= raddr0_;
        raddr1 <= raddr1_;
        raddr2 <= raddr2_;
        raddr3 <= raddr3_;
        if (wen0) begin
            data[waddr0] <= wdata0;
        end
        if (wen1) begin
            data[waddr1] <= wdata1;
        end
        if (wen2) begin
            data[waddr2] <= wdata2;
        end
        if (wen3) begin
            data[waddr3] <= wdata3;
        end
    end

endmodule
