`timescale 1ps/1ps

module main();

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(0,main);

    end

    // clock
    wire clk;
    clock c0(clk);

    reg[3:0] halt = 0;
    wire isHalt = halt>5;

    reg[15:0] pc=0;
    reg [15:0]f2_pc;
    reg [15:0]d_pc;
    reg [15:0]x1_pc;
    reg [15:0]x2_pc;
    reg [15:0]wb_pc;

    counter ctr(isHalt,clk);


    // read from memory
    wire[23:0] instruction;
    wire [15:0] out;
    wire [15:0]mem_raddr;
    wire[15:0] mem_loaded_data;
    wire mem_wen0;
    wire [15:0]mem_waddr;
    wire [7:0]mem_wdata0;
    wire mem_wen1;
    wire [7:0]mem_wdata1;
    wire pop;
    wire push;
    wire [15:0] stack_data;
    wire swap;
    wire replace_SP;


    mem memory(clk,pc,instruction,mem_raddr,mem_loaded_data,mem_wen0,mem_waddr,mem_wdata0, mem_wen1,mem_wdata1,pop, push, stack_data, swap, replace_SP, out);

    wire [2:0]reg_raddr0;
    wire[7:0]r_data0;
    wire [2:0]reg_raddr1;
    wire[7:0]r_data1;
    wire [2:0]reg_raddr2;
    wire[7:0]r_data2;
    wire [2:0]reg_raddr3;
    wire[7:0]r_data3;
    wire [2:0]reg_raddr4;
    wire[7:0]r_data4;
    wire [2:0]reg_raddr5;
    wire[7:0]r_data5;
    wire reg_wen0;
    wire reg_wen1;
    wire reg_wen2;
    wire reg_wen3;
    wire [2:0]reg_waddr0;
    wire [7:0]reg_wdata0;
    wire [2:0]reg_waddr1;
    wire [7:0]reg_wdata1;
    wire [2:0]reg_waddr2;
    wire [7:0]reg_wdata2;
    wire [2:0]reg_waddr3;
    wire [7:0]reg_wdata3;
    // registers
    regs registers(clk,reg_raddr0,r_data0,reg_raddr1,r_data1,reg_raddr2,r_data2,reg_raddr3,r_data3,reg_raddr4,r_data4,reg_raddr5,r_data5,reg_wen0,reg_wen1,reg_wen2,reg_wen3,reg_waddr0,reg_wdata0,reg_waddr1,reg_wdata1,reg_waddr2,reg_wdata2,reg_waddr3,reg_wdata3);


    // shift registers
    reg f1_v = 1'b1;
    reg f2_v = 1'b1;
    reg d_v = 1'b0;
    reg x1_v = 1'b0;
    reg x2_v = 1'b0;
    reg wb_v = 1'b0;

    // DECODE
    wire [23:0] d_instruction = instruction;
    wire [7:0] d_opcode = instruction[23:16];
    wire [7:0] d_lb = instruction[15:8];
    wire [7:0] d_hb = instruction[7:0];
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
    wire [2:0] d_reg_dest_cond_restart = d_opcode[5:3]; // destination register
    wire [2:0] d_reg_src = d_opcode[2:0]; // source register
    wire [1:0] d_reg_rp = d_opcode[5:4]; // register pair
    // instruction size
    wire d_is_one_byte = d_control[0] || d_control[7] || d_control[8] || d_control[9] || d_control[10] ||
                            d_control[12] || d_control[14] || d_control[16] || d_control[18] || d_control[19] ||
                            d_control[20] || d_control[21] || d_control[22] || d_control[23] || d_control[24] ||
                            d_control[26] || d_control[28] || d_control[30] || d_control[31] || d_control[32] || 
                            d_control[33] || d_control[34] || d_control[35] || d_control[36] || d_control[37] || 
                            d_control[38] || d_control[43] || d_control[44] || d_control[45] || d_control[46] || 
                            d_control[47] || d_control[48] || d_control[49] || d_control[50] || d_control[53] || 
                            d_control[54] || d_control[55] || d_control[56];
    wire d_is_two_bytes = d_control[1] || d_control[11] || d_control[13] || d_control[15] || d_control[17] ||
                            d_control[25] || d_control[27] || d_control[29] || d_control[51] || d_control[52];
    wire d_is_three_bytes = !(d_is_one_byte || d_is_two_bytes);
    // choose which registers to read
    // registers: RP 1, RP 2, destination/high, low
    assign reg_raddr0 = (d_reg_rp == 2'b00) ? 3'b000 : 3'b010; // B or D
    assign reg_raddr1 = (d_reg_rp == 2'b00) ? 3'b001 : 3'b011; // C or E
    assign reg_raddr2 = 3'b100; // H
    assign reg_raddr3 = 3'b101; // L 
    assign reg_raddr4 = 3'b111; // A
    assign reg_raddr5 = (d_control[18] || d_control[19]) ? d_reg_dest_cond_restart : 
                        d_reg_src; // destination or src


    // feeding wires into execute 1 stage
    reg [56:0] x1_control;
    reg [23:0] x1_instruction;

    wire [7:0] x1_rp1_val = r_data0;
    wire [7:0] x1_rp2_val = r_data1;
    wire [7:0] x1_regH_val = r_data2;
    wire [7:0] x1_regL_val = r_data3;
    wire [7:0] x1_accumulator_val = r_data4;
    wire [7:0] x1_source_destination_val=r_data5;

    // EXECUTE 1
    // loading things into memory
    assign mem_raddr = x1_control[7] ? {x1_rp1_val, x1_rp2_val} : 
                        (x1_control[3] || x1_control[5]) ? {d_hb, d_lb} :
                        {x1_regH_val, x1_regL_val}; // TODO: forward later
    // feeding wires into execute 2 stage
    reg [56:0] x2_control;
    reg [7:0] x2_rp1_val;
    reg [7:0] x2_rp2_val;
    reg [7:0] x2_regH_val;
    reg [7:0] x2_regL_val;
    reg [7:0] x2_accumulator_val;
    reg [7:0] x2_source_destination_val;
    reg [23:0] x2_instruction;

    // EXECUTE 2
    // feeding wires into writeback
    reg [56:0] wb_control;
    reg [7:0] wb_rp1_val;
    reg [7:0] wb_rp2_val;
    reg [7:0] wb_regH_val;
    reg [7:0] wb_regL_val;
    reg [7:0] wb_accumulator_val;
    reg [7:0] wb_source_destination_val;
    reg [23:0] wb_instruction;

    // instructions that change flags
    // TODO: auxillary flag will be updated later
    wire wb_edits_flags = (wb_control[10] || wb_control[11] || wb_control[12] || wb_control[13] ||
                            wb_control[14] || wb_control[15] || wb_control[16] || wb_control[17] ||
                            wb_control[18] || wb_control[19] || wb_control[23] || wb_control[24] || 
                            wb_control[25] || wb_control[26] || wb_control[27] || wb_control[28] || 
                            wb_control[29] || wb_control[30] || wb_control[31]) && wb_v;
    wire wb_edits_carry = (wb_control[10] || wb_control[11] || wb_control[12] || wb_control[13] ||
                            wb_control[14] || wb_control[15] || wb_control[16] || wb_control[17] ||
                            wb_control[22] || wb_control[23] || wb_control[24] || wb_control[25] || 
                            wb_control[26] || wb_control[27] || wb_control[28] || wb_control[29] || 
                            wb_control[30] || wb_control[31] || wb_control[32] || wb_control[33] ||
                            wb_control[34] || wb_control[35]) && wb_v;

    wire flushed = jump || return || subroutine || just_returned;

    assign push = ((x2_control[47] && x2_v) && !flushed) || subroutine;
    assign pop = (wb_control[48] && x2_v && !flushed) || return;
    assign swap = wb_control[49] && x2_v && !flushed;
    assign replace_SP = wb_control[50] && x2_v && !flushed;

    assign stack_data = (subroutine) ? wb_pc+2 :
                        (push) ? {x2_rp1_val, x2_rp2_val} : {x2_regH_val, x2_regL_val};
    
    // updated A value, updating normal register value, updating memory (store)
    // editing A register
    wire wb_edits_A = wb_control[3] || wb_control[7] || wb_control[10] || wb_control[11] || wb_control[12] || wb_control[13] || 
                        wb_control[14] || wb_control[15] || wb_control[16] || wb_control[17] || wb_control[24] || 
                        wb_control[25] || wb_control[26] || wb_control[27] || wb_control[28] || wb_control[29] || 
                        wb_control[30] || wb_control[31] || wb_control[32] || wb_control[33] || wb_control[34] || 
                        wb_control[35] || wb_control[36];
    // TODO: check using accumulator value and get from regs, if from src and is MMM use memory loaded value
    wire [7:0] wb_src_or_M_val = wb_instruction[18:16] == 3'b110 ? mem_loaded_data[15:8] : wb_source_destination_val;
    wire [7:0] wb_dest_or_M_val = wb_instruction[21:19] == 3'b110 ? mem_loaded_data[15:8] : wb_source_destination_val;
    wire [8:0] wb_A_val = (wb_control[3] || wb_control[7]) ? mem_loaded_data[15:8] : // LDA a: load A from memory
                            (wb_control[10] || wb_control[30]) ? wb_accumulator_val + wb_src_or_M_val : // ADD S: add register to A; CMP S: compare register with A
                            (wb_control[11] || wb_control[31]) ? wb_accumulator_val + wb_instruction[15:8] : // ADI #: add immediate to A; CPI #: compare immediate with A
                            wb_control[12] ? wb_accumulator_val + wb_src_or_M_val + flags[0] : // ADC S: add register to A with carry
                            wb_control[13] ? wb_accumulator_val + wb_instruction[15:8] + flags[0] : // ACI #: add immediate to A with carry
                            wb_control[14] ? wb_accumulator_val - wb_src_or_M_val : // SUB S: subtract register from A
                            wb_control[15] ? wb_accumulator_val - wb_instruction[15:8] : // SUI #: subtract immediate from A
                            wb_control[16] ? wb_accumulator_val - (wb_src_or_M_val + flags[0]) : // SBB S: subtract register from A with borrow
                            wb_control[17] ? wb_accumulator_val - (wb_instruction[15:8] + flags[0]) : // SBI #: subtract immediate from A with borrow
                            wb_control[24] ? wb_accumulator_val & wb_src_or_M_val : // ANA S: and register with A
                            wb_control[25] ? wb_accumulator_val & wb_instruction[15:8] : // ANI #: and immediate with A
                            wb_control[26] ? wb_accumulator_val | wb_src_or_M_val : // ORA S: or register with A
                            wb_control[27] ? wb_accumulator_val | wb_instruction[15:8] : // ORI #: or immediate with A
                            wb_control[28] ? wb_accumulator_val ^ wb_src_or_M_val : // XRA S: exclusive OR register with A
                            wb_control[29] ? wb_accumulator_val ^ wb_instruction[15:8] : // XRI #: exclusive or immediate with A
                            wb_control[32] ? (wb_accumulator_val * 2)%256 + wb_accumulator_val[7] : // RLC : rotate A left
                            wb_control[33] ? (wb_accumulator_val / 2) + wb_accumulator_val[0] * 128 : // RRC: rotate A right
                            wb_control[34] ? (wb_accumulator_val * 2) + flags[0] : // RAL: rotate A left through carry
                            wb_control[35] ? (wb_accumulator_val / 2) + flags[0] * 128 + wb_accumulator_val[0] * 256 : // RAR: rotate A right through carry
                            wb_control[36] ? ~wb_accumulator_val : // CMA: compliment A
                            9'b0;
    // editing any register
    wire wb_edits_regs = wb_control[0] || wb_control[1] || wb_control[18] || wb_control[19];
    wire [7:0] wb_regs_val = wb_control[0] ? wb_src_or_M_val : // MOV D, S: move register to register
                                wb_control[1] ? wb_instruction[15:8] : // MVI D, #: move immediate to register
                                wb_control[18] ? wb_dest_or_M_val + 1 : // INR D: increment register
                                wb_control[19] ? wb_dest_or_M_val - 1 : // DCR D: decrement register
                                8'b0;
    // editing any register pair
    wire wb_edits_rp = wb_control[2] || wb_control[9] || wb_control[20] || wb_control[21];
    wire [15:0] wb_rp_val = wb_control[2] ? mem_loaded_data : // LXI RP, #: load register pair immediate
                            wb_control[9] ? {wb_regH_val, wb_regL_val} : // XCHG: exchange DE and HL content
                            wb_control[20] ? {wb_rp1_val, wb_rp2_val} + 1 : // INX RP: increment register pair
                            wb_control[21] ? {wb_rp1_val, wb_rp2_val} - 1 : // DCX RP: decrement register pair
                            16'b0;
    // editing H:L registers
    wire wb_edits_hl = wb_control[5] || wb_control[9] || wb_control[22];
    wire [15:0] wb_hl_val = wb_control[5] ? mem_loaded_data : // LHLD: load H:L from memory
                            wb_control[9] ? {wb_rp1_val, wb_rp2_val} : // XCHG: exchange DE and HL content
                            wb_control[22] ? {wb_rp1_val, wb_rp2_val} + {wb_regH_val + wb_regL_val} : // DAD RP: add register pair to HL
                            16'b0;


    wire destination_is_M = wb_edits_regs && wb_instruction[21:19]==3'b110;

    // storing to memory
    assign mem_wen0 = (wb_control[4] || wb_control[6] || wb_control[8] || destination_is_M) && wb_v;
    assign mem_wen1 = wb_control[6] && wb_v;
    assign mem_waddr = ( wb_control[4] || wb_control[6]) ? {wb_instruction[7:0], wb_instruction[15:8]} : 
                        (destination_is_M) ? {wb_regH_val, wb_regL_val} : 
                        {wb_rp1_val, wb_rp2_val};
    assign mem_wdata0 = (wb_control[4] || wb_control[8]) ? wb_accumulator_val :
                        (destination_is_M) ? wb_regs_val: 
                        wb_regH_val;
    assign mem_wdata1 = wb_regL_val;

    // condition
    wire [2:0] condition = wb_instruction[21:19];
    wire condition_is_true = (condition == 0 && !flags[6]) || (condition == 1 && flags[6]) || (condition == 2 && !flags[0]) || (condition == 3 && flags[0]) || (condition == 4 && !flags[2]) || (condition == 5 && flags[2]) || (condition == 6 && !flags[7]) || (condition == 7 && flags[7]);

    // jumping
    wire [15:0] jump_location = (wb_control[45]) ? wb_instruction[21:19]*8 :
                                (wb_control[46]) ? {wb_regH_val, wb_regL_val} :
                                {wb_instruction[7:0], wb_instruction[15:8]};
    wire jump = (wb_control[39] || wb_control[46] ||(wb_control[40] && condition_is_true)) && wb_v;
    wire subroutine = (wb_control[41] || wb_control[45] ||(wb_control[42] && condition_is_true)) && wb_v;

    //returning
    wire return = (wb_control[43] || (wb_control[44] && condition_is_true)) && wb_v;
    reg just_returned;

    // actual value to be written back to register or memory
    wire [15:0] wb_val = wb_edits_A ? wb_A_val[7:0] *256 :
                        wb_edits_regs ? wb_regs_val*256 :
                        wb_edits_hl ? wb_hl_val :
                        wb_edits_rp ? wb_rp_val : 
                        0;


    assign reg_wen0 = (wb_edits_A || (wb_edits_regs && destination_is_M) || wb_edits_hl || wb_edits_rp) && wb_v;
    assign reg_wen1 = (wb_edits_hl || wb_edits_rp) && wb_v;
    assign reg_wen2 = wb_control[9] & wb_v;
    assign reg_wen3 = wb_control[9] & wb_v;

    assign reg_waddr0 = (wb_edits_A) ? 3'b111 :
                        (wb_edits_regs) ? wb_instruction[21:19] :
                        (wb_edits_hl) ? 3'b100 :
                        (wb_edits_rp && wb_instruction[21:20]==2'b00) ? 3'b000 : 
                        (wb_edits_rp && wb_instruction[21:20]==2'b01) ? 3'b010 : 
                        0;

    assign reg_waddr1 = (wb_edits_hl) ? 3'b101 :
                        (wb_edits_rp && wb_instruction[21:20]==2'b00) ? 3'b001 : 
                        (wb_edits_rp && wb_instruction[21:20]==2'b01) ? 3'b011 : 
                        0;

    assign reg_waddr2 = (wb_instruction[21:20]==2'b00) ? 3'b000 : 
                        (wb_instruction[21:20]==2'b01) ? 3'b010 :
                        0;

    assign reg_waddr3 = (wb_instruction[21:20]==2'b00) ? 3'b001 : 
                        (wb_instruction[21:20]==2'b01) ? 3'b011 :
                        0;

    assign reg_wdata0 = wb_val[15:8];
    assign reg_wdata1 = wb_val[7:0];
    assign reg_wdata2 = wb_rp_val[15:8];
    assign reg_wdata3 = wb_rp_val[7:0];



  
    // CARRY FLAGS
    reg [7:0] flags; // sign, zero, 0, auxillary carry, 0, parity, 1, carry
    // TODO: when setting carry flag, if addition, take top bit; if subtraction and result is 1, take the reverse of the current carry flag

    always @(posedge clk) begin
        if(wb_control[55] && wb_v)begin
            halt<=1;
        end

        // feeding wires from decode to execute 1
        x1_control <= d_control;
        x1_instruction <= d_instruction;

        // feeding wires from execute 1 to execute 2
        x2_control <= x1_control;
        x2_rp1_val <= x1_rp1_val;
        x2_rp2_val <= x1_rp2_val;
        x2_regH_val <= x1_regH_val;
        x2_regL_val <= x1_regL_val;
        x2_accumulator_val <= x1_accumulator_val;
        x2_source_destination_val <= x1_source_destination_val;
        x2_instruction <= x1_instruction;

        // feeding wires from execute 2 to writeback
        wb_control <= x2_control;
        wb_rp1_val <= x2_rp1_val;
        wb_rp2_val <= x2_rp2_val;
        wb_regH_val <= x2_regH_val;
        wb_regL_val <= x2_regL_val;
        wb_accumulator_val <= x2_accumulator_val;
        wb_source_destination_val <= x2_source_destination_val;
        wb_instruction <= x2_instruction;

        // updating flags:
        // CMC: compliment carry flag
        if (wb_control[37] && wb_v) begin
            flags[0] <= ~flags[0];
        end
        else if (wb_control[38] && wb_v) begin
            flags[0] <= 1;
        end
        else begin
            if (wb_edits_flags) begin
                flags[5] <= wb_val == 0; // setting ZERO flag
                flags[6] <= wb_val[7] == 1; // setting SIGN flag
                flags[2] <= (wb_val[0] + wb_val[1] + wb_val[2] + wb_val[3] + wb_val[4] + wb_val[5] + wb_val[6] + wb_val[7] + 1) % 2; // check PARITY flag
            end
            if (wb_edits_carry) begin
                flags[0] <= wb_val[9]; // setting CARRY flag
            end
        end

        just_returned<=return;

        // shift registers
        if (flushed) begin
            if (return) begin
                f1_v<=0;
            end
            else if (just_returned) begin
                f1_v<=1;
                pc<=out;
            end
            else begin
                pc<=jump_location;

            end
            f2_v <= 0;
            d_v<=0;
            x1_v <= 0;
            x2_v <= 0;
            wb_v <= 0;
        end
        else begin 
            if(d_is_two_bytes && d_v) begin
                f2_v <= f1_v;
                d_v<=0;
                x1_v <= d_v;
                x2_v <= x1_v;
                wb_v <= x2_v;
                pc<=pc+1;
            end
            if(d_is_three_bytes && d_v) begin
                f2_v <= 0;
                d_v<=0;
                x1_v <= d_v;
                x2_v <= x1_v;
                wb_v <= x2_v;
                pc<=pc+1;
            end
            else begin
                f2_v <= f1_v;
                d_v <= f2_v;
                x1_v <= d_v;
                x2_v <= x1_v;
                wb_v <= x2_v;
                pc<=pc+1;
            end
        end

        f2_pc<=pc;
        d_pc<=f2_pc;
        x1_pc<=d_pc;
        x2_pc<=x1_pc;
        wb_pc<=x2_pc;

        // check if its one or two or three bytes and adjust pc and shift registers

        if(wb_control[52] && wb_v) begin
            $write("%c",(wb_val&8'b11111111));
        end
        halt<=halt+1;
    end


endmodule
