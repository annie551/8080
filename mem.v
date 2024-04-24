`timescale 1ps/1ps

module mem(input clk,
    input [15:0]raddr0_, output [15:0]rdata0_,
    input [15:0]raddr1_, output [15:0]rdata1_,
    input wen, input [15:1]waddr, input [15:0]wdata,
    input pop, input push, input[15:0] pushorswap_data, input swap,
    output out);

    wire out;

    reg [7:0]data[0:16'hffff];

    reg[15:0] stack_top=16'hffff;

    /* Simulation -- read initial content from file */
    initial begin
        $readmemh("mem.hex",data);
    end

    reg [15:0]raddr0;
    reg [15:0]rdata0;

    reg [15:0]raddr1;
    reg [15:0]rdata1;

    assign rdata0_ = rdata0;
    assign rdata1_ = rdata1;

    reg[15:0] data_out;
    wire [15:0] out = data_out;

    always @(posedge clk) begin
        raddr0 <= raddr0_;
        raddr1 <= raddr1_;
        rdata0 <= data[raddr0];
        rdata1 <= data[raddr1];
        if (wen) begin
            data[waddr] <= wdata;
        end

        if(push) begin
            stack_top<=stack_top-2;
            data[stack_top]<=pushorswap_data[7:0];
            data[stack_top-1]<=pushorswap_data[15:8];
        end

        if(pop) begin
            stack_top<=stack_top+2;
            data_out<={data[stack_top+1],data[stack_top+2]};
        end

        if(swap) begin
            data[stack_top+2]<=pushorswap_data[7:0];
            data[stack_top+1]<=pushorswap_data[15:8];
            data_out<={data[stack_top+1],data[stack_top+2]};
        end

        if(wen && waddr>16'hbfff) begin
            $write("out of range of memory allocation");
        end

        if(push && stack_top<16'hbfff) begin
            $write("stack space exceeded");
        end


    end

endmodule
