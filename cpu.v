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

    reg[15:0] pc=0;

    counter ctr(halt,clk);


    // read from memory
    wire[15:0] ins;
    wire [15:0] instruction;
    wire [15:0]mem_raddr1;
    wire[15:0] mem_load;
    wire mem_wen;
    wire [15:0]mem_waddr;
    wire [15:0]mem_wdata;
    wire [15:0] out;

    mem memory(clk,pc[15:1],ins,mem_raddr1[15:1],mem_load,mem_wen,mem_waddr[15:1],mem_wdata[15:0], mem_wen, mem_wen, pc, mem_wen, out[15:0]);

    
    wire [3:0]reg_raddr0;
    wire[15:0]ra_init;
    wire [3:0]reg_raddr1;
    wire[15:0]rt_init;
    wire reg_wen;
    wire [3:0]reg_waddr;
    wire [15:0]reg_wdata;
    // registers
    regs registers(clk,reg_raddr0[3:0],ra_init[15:0],reg_raddr1[3:0],rt_init[15:0],reg_wen,reg_waddr[3:0],reg_wdata[15:0]);

    // DECODE
    wire [7:0] d_opcode = ins[23:16];
    // control wires
    wire d_mov = (d_opcode[7:6] == 2'b01);
    wire d_mvi = (d_opcode[7:6] == 2'b00) && (d_opcode[2:0] == 3'b110);
    wire d_lxi = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b0001);
    wire d_lda = (d_opcode[7:0] == 8'b00111010);
    wire d_sta = (d_opcode[7:0] == 8'b00110010);
    wire d_lhld = (d_opcode[7:0] == 8'b00101010);
    wire d_shld = (d_opcode[7:0] == 8'b00100010);
    wire d_ldax = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b1010);
    wire d_stax = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b0010);
    wire d_xchg = (d_opcode[7:0] == 8'b11101011);
    wire d_add = (d_opcode[7:3] == 5'b10000);
    wire d_adi = (d_opcode[7:0] == 8'b11000110);
    wire d_adc = (d_opcode[7:3] == 5'b10001);
    wire d_aci = (d_opcode[7:0] == 8'b11001110);
    wire d_sub = (d_opcode[7:3] == 5'b10010);
    wire d_sui = (d_opcode[7:0] == 8'b11010110);
    wire d_sbb = (d_opcode[7:3] == 5'b10011);
    wire d_sbi = (d_opcode[7:0] == 8'b11011110);
    wire d_inr = (d_opcode[7:6] == 2'b00) && (d_opcode[2:0] == 3'b100);
    wire d_dcr = (d_opcode[7:6] == 2'b00) && (d_opcode[2:0] == 3'b101);
    wire d_inx = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b0011);
    wire d_dcx = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b1011);
    wire d_dad = (d_opcode[7:6] == 2'b00) && (d_opcode[3:0] == 4'b1001);
    wire d_daa = (d_opcode == 8'b00100111);
    wire d_ana = (d_opcode[7:5] == 3'b10100);
    wire d_ani = (d_opcode == 8'b11100110);
    wire d_ora = (d_opcode[7:3] == 3'b10110);
    wire d_ori = (d_opcode == 8'b11110110);
    wire d_xra = (d_opcode[7:3] == 5'b10101);
    wire d_xri = (d_opcode == 8'b11101110);
    wire d_cmp = (d_opcode[7:3] == 3'b10111);
    wire d_cpi = (d_opcode == 8'b11111110);
    wire d_rlc = (d_opcode == 8'b00000111);
    wire d_rrc = (d_opcode == 8'b00001111);
    wire d_ral = (d_opcode == 8'b00010111);
    wire d_rar = (d_opcode == 8'b00011111);
    wire d_cma = (d_opcode == 8'b00101111);
    wire d_cmc = (d_opcode == 8'b00111111);
    wire d_stc = (d_opcode == 8'b00110111);
    wire d_jmp = (d_opcode == 8'b11000011);
    wire d_jccc = (d_opcode[7:5] == 3'b11) && (d_opcode[2:0] == 3'b010);
    wire d_call = (d_opcode == 8'b11001101);
    wire d_cccc = (d_opcode[7:6] == 2'b11) && (d_opcode[2:0] == 3'b100);
    wire d_ret = (d_opcode == 8'b11001001);
    wire d_rccc = (d_opcode[7:6] == 2'b11) && (d_opcode[3:0] == 3'b000);
    wire d_rst = (d_opcode[7:6] == 2'b11) && (d_opcode[2:0] == 3'b111);
    wire d_pchl = (d_opcode == 8'b11101001);
    wire d_push = (d_opcode[7:6] == 2'b11) && (d_opcode[3:0] == 4'b0101);
    wire d_pop = (d_opcode[7:6] == 2'b11) && (d_opcode[3:0] == 4'b0001);
    wire d_xthl = (d_opcode == 8'b11100011);
    wire d_sphl = (d_opcode == 8'b11111001);
    wire d_in_p = (d_opcode == 8'b11011011);
    wire d_out_p = (d_opcode == 8'b11010011);
    wire d_ei = (d_opcode == 8'b11111011);
    wire d_di = (d_opcode == 8'b11110011);
    wire d_hlt = (d_opcode == 8'b01110110);
    wire d_nop = (d_opcode == 8'b00000000);

    always @(posedge clk) begin
        // if(NotValid)begin
        //     halt<=1;
        // end
        pc<=pc+2;
        halt<=1;

        // if(print && halt==0) begin
        //     $write("%c",(reg_wdata&8'b11111111));
        // end
    end


endmodule
