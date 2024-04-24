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
    wire[23:0] instruction;
    wire [15:0] out;
    wire [15:0]mem_raddr;
    wire[15:0] mem_loaded_data;
    wire mem_wen;
    wire [15:0]mem_waddr;
    wire [15:0]mem_wdata;
    wire pop;
    wire push;
    wire [15:0] stack_data;
    wire swap;
    wire replace_SP;


    mem memory(clk,pc,instruction,mem_raddr,mem_loaded_data,mem_wen,mem_waddr,mem_wdata, pop, push, stack_data, swap, replace_SP, out);

    wire [2:0]reg_raddr0;
    wire[7:0]r_data0;
    wire [2:0]reg_raddr1;
    wire[7:0]r_data1;
    wire [2:0]reg_raddr2;
    wire[7:0]r_data2;
    wire [2:0]reg_raddr3;
    wire[7:0]r_data3;
    wire reg_wen;
    wire [2:0]reg_waddr0;
    wire [7:0]reg_wdata0;
    wire [2:0]reg_waddr1;
    wire [7:0]reg_wdata1;
    wire [2:0]reg_waddr2;
    wire [7:0]reg_wdata2;
    wire [2:0]reg_waddr3;
    wire [7:0]reg_wdata3;
    // registers
    regs registers(clk,reg_raddr0,r_data0,reg_raddr1,r_data1,reg_raddr2,r_data2,reg_raddr3,r_data3,reg_wen,reg_waddr0,reg_wdata0,reg_waddr1,reg_wdata1,reg_waddr2,reg_wdata2,reg_waddr3,reg_wdata3);


    // shift registers
    reg f1_v = 1'b1;
    reg f2_v = 1'b1;
    reg d_v = 1'b0;
    reg x1_v = 1'b0;
    reg x2_v = 1'b0;
    reg wb_v = 1'b0;

    // DECODE
    wire [7:0] d_opcode = instruction[23:16];
    // register opcodes
    wire [2:0] regA = 3'b111;
    wire [2:0] regB = 3'b000;
    wire [2:0] regC = 3'b001;
    wire [2:0] regD = 3'b010;
    wire [2:0] regE = 3'b011;
    wire [2:0] regH = 3'b100;
    wire [2:0] regL = 3'b101;
    wire [2:0] regM = 3'b110;
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
    wire d_ana = (d_opcode[7:3] == 5'b10100);
    wire d_ani = (d_opcode == 8'b11100110);
    wire d_ora = (d_opcode[7:3] == 5'b10110);
    wire d_ori = (d_opcode == 8'b11110110);
    wire d_xra = (d_opcode[7:3] == 5'b10101);
    wire d_xri = (d_opcode == 8'b11101110);
    wire d_cmp = (d_opcode[7:3] == 5'b10111);
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
    // final control wire
    wire [56:0] d_control = {d_nop, d_hlt, d_di, d_ei, d_out_p, d_in_p, d_sphl, d_xthl, d_pop, d_push,
                   d_pchl, d_rst, d_rccc, d_ret, d_cccc, d_call, d_jccc, d_jmp, d_stc, d_cmc,
                   d_cma, d_rar, d_ral, d_rrc, d_rlc, d_cpi, d_cmp, d_xri, d_xra, d_ori, d_ora,
                   d_ani, d_ana, d_daa, d_dad, d_dcx, d_inx, d_dcr, d_inr, d_sbi, d_sbb, d_sui,
                   d_sub, d_aci, d_adc, d_adi, d_add, d_xchg, d_stax, d_ldax, d_shld, d_lhld,
                   d_sta, d_lda, d_lxi, d_mvi, d_mov};
    // registers to read
    wire [2:0] d_reg_dest = d_opcode[5:3]; // destination register
    wire [2:0] d_reg_src = d_opcode[2:0]; // source register
    wire [1:0] d_reg_rp = d_opcode[5:4]; // register pair
    // instruction size
    wire d_is_one_byte = d_control[0] || d_control[7] || d_control[8] || d_control[9] || d_control[10] ||
                            d_control[12] || d_control[14] || d_control[16] || d_control[18] || d_control[19] ||
                            d_control[20] || d_control[21] || d_control[22] || d_control[23] || d_control[24] ||
                            d_control[26] || d_control[27] || d_control[28] || d_control[30] || d_control[31] ||
                            d_control[32] || d_control[33] || d_control[34] || d_control[35] || d_control[36] ||
                            d_control[37] || d_control[38] || d_control[43] || d_control[44] || d_control[45] ||
                            d_control[46] || d_control[47] || d_control[48] || d_control[49] || d_control[50] ||
                            d_control[53] || d_control[54] || d_control[55] || d_control[56];
    wire d_is_two_bytes = d_control[1] || d_control[11] || d_control[13] || d_control[15] || d_control[17] ||
                            d_control[25] || d_control[29] || d_control[51] || d_control[52];
    wire d_is_three_bytes = !(d_is_one_byte || d_is_two_bytes);

    
    // choose which registers to read
    // registers: RP 1, RP 2, destination/high, low
    wire [2:0] d_reg_pair1 = (d_reg_rp == 2'b00) ? 3'b000 : 3'b010;
    wire [2:0] d_reg_pair2 = (d_reg_rp == 2'b00) ? 3'b001 : 3'b011;
    wire d_uses_hl = d_control[5] || d_control[6] || d_control[9] || d_control[22] || d_control[46] ||
                        d_control[49] || d_control[50];
    wire [2:0] d_reg_dest_high = d_uses_hl ? 3'b100 : d_reg_dest;
    wire [2:0] d_reg_low = 3'b101;
    // feeding wires into execute 1 stage




    always @(posedge clk) begin
        // if(NotValid)begin
        //     halt<=1;
        // end

        // shift registers
        f2_v <= f1_v;
        d_v <= f2_v;
        x1_v <= d_v;
        x2_v <= x1_v;
        wb_v <= x2_v;

        // check if its one or two or three bytes and adjust pc and shift registers

        pc<=pc+2;
        halt<=1;

        // if(print && halt==0) begin
        //     $write("%c",(reg_wdata&8'b11111111));
        // end
    end


endmodule
