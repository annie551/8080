`timescale 1ps/1ps

module mem(input clk,
    input [15:0]raddr0_, output [23:0]rdata0_,
    input [15:0]raddr1_, output [15:0]rdata1_,
    input wen0, input [15:0]waddr, input [7:0]wdata0,
    input wen1, input [7:0]wdata1,
    input pop, input push, input[15:0] input_data, input swap, input replace_SP,
    output[15:0] out);

    wire out;

    reg [7:0]data[0:16'hffff];

    reg[16:0] stack_top=20'h10000;

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
    assign out = data_out;

    always @(posedge clk) begin
        raddr0 <= raddr0_;
        raddr1 <= raddr1_;
        rdata0 <= {data[raddr0],data[raddr0+1],data[raddr0+2]};  //reads in instructions 3 bytes at a time
        rdata1 <= {data[raddr1], data[raddr1+1]};   //loads two bytes at a time
        if (wen0) begin
            data[waddr] <= wdata0;
        end
        if (wen1) begin
            data[waddr+1] <= wdata1;
        end

        if(push) begin
            stack_top<=stack_top-2;
            data[stack_top-1]<=input_data[15:8];
            data[stack_top-2]<=input_data[7:0];
        end

        if(pop) begin
            stack_top<=stack_top+2;
            data_out<={data[stack_top+1],data[stack_top]};
        end

        if(swap) begin
            data[stack_top]<=input_data[7:0];
            data[stack_top+1]<=input_data[15:8];
            data_out<={data[stack_top+1],data[stack_top]};
        end

        if((wen0 || wen1) && waddr>16'hbfff) begin
            $write("out of range of memory allocation");
        end

        if(push && stack_top<16'hbfff) begin
            $write("stack space exceeded");
        end

        if(replace_SP) begin
            stack_top<=input_data;
        end


    end

endmodule
