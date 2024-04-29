#!/usr/bin/env python3

import sys

if len(sys.argv) == 1:
    subcommand = input("Select an operation from [asm, disasm]: ")
    f = input("File path: ")
else:
    subcommand = sys.argv[1]
    f = sys.argv[2]
    
if subcommand == "asm":
    with open(f,"r") as file:
        for line in file.readlines():
            if len(line) <= 1:
                continue
            jmps = ["jz", "jnz", "js", "jns"]
            line = line.lower()
            if line[0] == "@" or line[0:2] == "//":
                print(line[:-1])
                continue
            args = line.split()
            if "," in args:
                args.remove(",")
            # item = 0
            # while item < len(args):
            #     args[item] = args[item].strip()
            #     item += 1
            op = args[0]
            rp_ins = ["ldax", "stax", "inx", "dcx", "dad", "push", "pop", "lxi"]
            dest_ins = ["mov", "mvi", "inr", "dcr"]
            src_ins = ["add", "adc", "sub", "sbb", "ana", "ora", "xra", "cmp"]
            db_ins = ["adi", "aci", "sui", "sbi", "ani", "xri", "in", "out"]
            lbhb_ins = ["lda", "sta", "lhld", "shld", "jmp", "jnz", "jz", "jnc", "jc", "jpo", "jpe", "jp", "jm", "call", "cnz", "cz", "cnc", "cc", "cpo", "cpe", "cp", "cm"]
            ret_ins = ["rnz", "rz", "rnc", "rc", "rpo", "rpe", "rp", "rm"]

            # setting dummy vars
            imm = dest = src = 0


            if op == "xchg":
                instr = 0b11101011
            elif op == "daa":
                instr = 0b00100111
            elif op == "ori":
                instr = 0b11110110
            elif op == "cpi":
                instr = 0b11111110
            elif op == "rlc":
                instr = 0b00000111
            elif op == "rrc":
                instr = 0b00001111
            elif op == "ral":
                instr = 0b00010111
            elif op == "rar":
                instr = 0b00011111
            elif op == "cma":
                instr = 0b00101111
            elif op == "cmc":
                instr = 0b00111111
            elif op == "stc":
                instr = 0b00110111
            elif op == "ret":
                instr = 0b11001001
            elif op == "pchl":
                instr = 0b11101001
            elif op == "xthl":
                instr = 0b11100011
            elif op == "sphl":
                instr = 0b11111001
            elif op == "ei":
                instr = 0b11111011
            elif op == "di":
                instr = 0b11110011
            elif op == "hlt":
                instr = 0b01110110
            elif op == "nop":
                instr = 0b00000000
            elif op in rp_ins:
                rp_letters = args[1][0]
                if rp_letters == "bc":
                    rp = 0b00
                elif rp_letters == "de":
                    rp = 0b01
                elif rp_letters == "hl":
                    rp = 0b10
                elif rp_letters == "sp":
                    rp = 0b11
                if op == "ldax":
                    instr = 0b00 << 6 | rp << 4 | 0b1010
                elif op == "stax":
                    instr = 0b00 << 6 | rp << 4 | 0b0010
                elif op == "inx":
                    instr = 0b00 << 6 | rp << 4 | 0b0011
                elif op == "dcx":
                    instr = 0b00 << 6 | rp << 4 | 0b1011
                elif op == "dad":
                    instr = 0b00 << 6 | rp << 4 | 0b1001
                elif op == "push":
                    instr = 0b11 << 6 | rp << 4 | 0b0101
                elif op == "pop":
                    instr = 0b11 << 6 | rp << 4 | 0b0001
                elif op == "lxi":
                    imm = int(args[2])
                    instr = 0b00 << 14 | rp << 12 | 0b0001 << 8 | (imm & 0xFF) | ((imm >> 8) & 0xFF)
            elif op in dest_ins:
                dest_letters = args[1][0]
                if dest_letters == "a":
                    dest = 0b111
                elif dest_letters == "b":
                    dest = 0b000
                elif dest_letters == "c":
                    dest = 0b001
                elif dest_letters == "d":
                    dest = 0b010
                elif dest_letters == "e":
                    dest = 0b011
                elif dest_letters == "h":
                    dest = 0b100
                elif dest_letters == "l":
                    dest = 0b101
                elif dest_letters == "m":
                    dest = 0b110
                if op == "mov":
                    src_letters = args[2]
                    if src_letters == "a":
                        src = 0b111
                    elif src_letters == "b":
                        src = 0b000
                    elif src_letters == "c":
                        src = 0b001
                    elif src_letters == "d":
                        src = 0b010
                    elif src_letters == "e":
                        src = 0b011
                    elif src_letters == "h":
                        src = 0b100
                    elif src_letters == "l":
                        src = 0b101
                    elif src_letters == "m":
                        src = 0b110     
                    instr = 0b01 << 6 | dest << 3 | src
                elif op == "mvi":
                    imm = int(args[2])
                    instr = 0b00 << 14 | dest << 11 | 0b110 << 8 | (imm & 0xFF)
                elif op == "inr":
                    instr = 0b00 << 6 | dest << 3 | 0b100
                elif op == "dcr":
                    instr = 0b00 << 6 | dest << 3 | 0b101
            elif op in src_ins:
                src_letters = args[1][0]
                if src_letters == "a":
                    src = 0b111
                elif src_letters == "b":
                    src = 0b000
                elif src_letters == "c":
                    src = 0b001
                elif src_letters == "d":
                    src = 0b010
                elif src_letters == "e":
                    src = 0b011
                elif src_letters == "h":
                    src = 0b100
                elif src_letters == "l":
                    src = 0b101
                elif src_letters == "m":
                    src = 0b110
                if op == "add":
                    instr = 0b10000 << 3 | src
                elif op == "adc":
                    instr = 0b10001 << 3 | src
                elif op == "sub":
                    instr = 0b10010 << 3 | src
                elif op == "sbb":
                    instr = 0b10011 << 3 | src
                elif op == "ana":
                    instr = 0b10100 << 3 | src
                elif op == "ora":
                    instr = 0b10110 << 3 | src
                elif op == "xra":
                    instr = 0b10101 << 3 | src
                elif op == "cmp":
                    instr = 0b10111 << 3 | src
            elif op in db_ins:
                db = int(args[1]) & 0xFF
                if op == "adi":
                    instr = 0b11000110 << 8 | db
                elif op == "aci":
                    instr = 0b11001110 << 8 | db
                elif op == "sui":
                    instr = 0b11010110 << 8 | db
                elif op == "sbi":
                    instr = 0b11011110 << 8 | db
                elif op == "ani":
                    instr = 0b11100110 << 8 | db
                elif op == "xri":
                    instr = 0b11101110 << 8 | db
                elif op == "in":
                    instr = 0b11011011 << 8 | db
                elif op == "out":
                    instr = 0b11010011 << 8 | db
            elif op in lbhb_ins:
                lb = int(args[1]) & 0xFF
                hb = (int(args[1]) >> 8) & 0xFF
                db = lb << 8 | hb
                if op == "lda":
                    instr = 0b00111010 << 16 | db
                elif op == "sta":
                    instr = 0b00110010 << 16 | db
                elif op == "lhld":
                    instr = 0b00101010 << 16 | db
                elif op == "shld":
                    instr = 0b001000010 << 16 | db
                elif op == "jmp":
                    instr = 0b11000011 << 16 | db
                elif op == "jnz":
                    instr = 0b11000010 << 16 | db
                elif op == "jz":
                    instr = 0b11001010 << 16 | db
                elif op == "jnc":
                    instr = 0b11010010 << 16 | db
                elif op == "jc":
                    instr = 0b11011010 << 16 | db
                elif op == "jpo":
                    instr = 0b11100010 << 16 | db
                elif op == "jpe":
                    instr = 0b11101010 << 16 | db
                elif op == "jp":
                    instr = 0b11110010 << 16 | db
                elif op == "jm":
                    instr = 0b11111010 << 16 | db
                elif op == "call":
                    instr = 0b11001101 << 16 | db
                elif op == "cnz":
                    instr = 0b11000100 << 16 | db
                elif op == "cz":
                    instr = 0b11001100 << 16 | db
                elif op == "cnc":
                    instr = 0b11010100 << 16 | db
                elif op == "cc":
                    instr = 0b11011100 << 16 | db
                elif op == "cpo":
                    instr = 0b11100100 << 16 | db
                elif op == "cpe":
                    instr = 0b11101100 << 16 | db
                elif op == "cp":
                    instr = 0b11110100 << 16 | db
                elif op == "cm":
                    instr = 0b11111100 << 16 | db
            elif op in ret_ins:
                if op == "rnz":
                    instr = 0b11000000
                elif op == "rz":
                    instr = 0b11001000
                elif op == "rnc":
                    instr = 0b11010000
                elif op == "rc":
                    instr = 0b11011000
                elif op == "rpo":
                    instr = 0b11100000
                elif op == "rpe":
                    instr = 0b11101000
                elif op == "rp":
                    instr = 0b11110000
                elif op == "rm":
                    instr = 0b11111000
            elif op == "rst":
                n = int(args[1]) & 0xF
                instr = 0b11 << 6 | n << 3 | 0b111
            else:
                print("Unknown instruction:", op)
                continue
            
            print("%0.4x    // %s" % (instr, line[:-1]))

else:
    print("Bad subcommand option:", subcommand)
