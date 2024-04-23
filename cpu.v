`timescale 1ps/1ps

module main();

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(0,main);
        
    end

    // clock
    wire clk;
    clock c0(clk);

    reg halt = 0;

    counter ctr(halt,clk);


    // read from memory
    wire[15:0] ins;
    wire [15:0] instruction;
    wire [15:0]mem_raddr1;
    wire[15:0] mem_load;
    wire mem_wen;
    wire [15:0]mem_waddr;
    wire [15:0]mem_wdata;

    mem memory(clk,pc[15:1],ins,mem_raddr1[15:1],mem_load,mem_wen,mem_waddr[15:1],mem_wdata[15:0]);


    wire [3:0]reg_raddr0;
    wire[15:0]ra_init;
    wire [3:0]reg_raddr1;
    wire[15:0]rt_init;
    wire reg_wen;
    wire [3:0]reg_waddr;
    wire [15:0]reg_wdata;
    // registers
    regs registers(clk,reg_raddr0[3:0],ra_init[15:0],reg_raddr1[3:0],rt_init[15:0],reg_wen,reg_waddr[3:0],reg_wdata[15:0]);




    always @(posedge clk) begin
        if(NotValid)begin
            halt<=1;
        end
        pc<=pc+2;


        if(print && halt==0) begin
            $write("%c",(reg_wdata&8'b11111111));
        end
    end


endmodule
