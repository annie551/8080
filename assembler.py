#!/usr/bin/env python3

import sys

if len(sys.argv) == 1:
    subcommand = input("Select an operation from [asm, disasm]: ")
    f = input("File path: ")
else:
    subcommand = sys.argv[1]
    f = sys.argv[2]
    
instructions = []
pc = 0
labels = {}
labels_to_update = {}
line_counter = 0

assembly_instructions = [
    "nop", "lxi", "stax", "inx", "inr", "dcr", "mvi", "rlc", "dad", "ldax",
    "dcx", "rrc", "ral", "rar", "rim", "shld", "daa", "lhld", "cma", "sim",
    "sta", "stc", "lda", "cmc", "mov", "hlt", "add", "adc", "sub", "sbb",
    "ana", "xra", "ora", "cmp", "rnz", "pop", "jnz", "jmp", "cnz", "push",
    "adi", "rst", "rz", "ret", "jz", "cz", "call", "aci", "rnc", "jnc", "out",
    "cnc", "sui", "rc", "jc", "in", "cc", "sbi", "rpo", "jpo", "xthl", "cpo",
    "ani", "rpe", "pchl", "jpe", "xchg", "cpe", "xri", "rp", "jp", "di", "cp",
    "ori", "rm", "sphl", "jm", "ei", "cm", "cpi"
]
rp_ins = ["ldax", "stax", "inx", "dcx", "dad", "push", "pop", "lxi"]
dest_ins = ["mov", "mvi", "inr", "dcr"]
src_ins = ["add", "adc", "sub", "sbb", "ana", "ora", "xra", "cmp"]
db_ins = ["adi", "aci", "sui", "sbi", "ani", "xri", "in", "out"]
lbhb_ins = ["lda", "sta", "lhld", "shld", "jmp", "jnz", "jz", "jnc", "jc", "jpo", "jpe", "jp", "jm", "call", "cnz", "cz", "cnc", "cc", "cpo", "cpe", "cp", "cm"]
ret_ins = ["rnz", "rz", "rnc", "rc", "rpo", "rpe", "rp", "rm"]
one_byte = [
    "mov", "ldax", "stax", "xchg", "add", "adc", "sub", "sbb", 
    "inr", "dcr", "inx", "dcx", "dad", "daa", "ana", "ora", 
    "xra", "cmp", "cpi", "rlc", "rrc", "ral", "rar", "cma", 
    "cmc", "stc", "ret", "rnz", "rz", "rnc", "rc", "rpo", 
    "rpe", "rp", "rm", "rst", "pchl", "push", "pop", "xthl", 
    "sphl", "ei", "di", "hlt", "nop"
]
two_byte = ["mvi", "adi", "aci", "sui", "sbi", "ani", "ori", "xir", "in", "out"]
three_byte = ["lxi", "lda", "sta", "lhld", "shld", "jmp", "jnz", "jz", "jc", "jnc", "jpo", "jpe", "jp", "jm", "call", "cnz", "cz", "cnc", "cc", "cpo", "cpe", "cp", "cm"]

if subcommand == "asm":
    with open(f,"r") as file:
        for line in file.readlines():
            if len(line) <= 1:
                continue
            line = line.lower()
            if line[0] == "@" or line[0:2] == "//":
                print(line[:-1])
                continue
            args = line.split()
            if "," in args:
                args.remove(",")

            op = args[0]

            # setting dummy vars
            imm = dest = src = 0

            # labels

            if op not in assembly_instructions:
                op = op[:-1]
                lb = pc & 0xFF
                hb = (pc >> 8) & 0xFF
                db = format((lb << 8 | hb), '04x')
                labels[op] = db
                if op in labels_to_update:
                    # print("instr:", instructions[labels_to_update[op]])
                    instructions[labels_to_update[op]] = instructions[labels_to_update[op]][:-4]
                    # print("instr:", instructions[labels_to_update[op]])
                    instructions[labels_to_update[op]] += str(db)
                    # print("instr:", instructions[labels_to_update[op]])

                    del labels_to_update[op]
                continue
            
            if op == "xchg":
                instr = 0b11101011
                pc += 1
            elif op == "daa":
                instr = 0b00100111
                pc += 1
            elif op == "ori":
                instr = 0b11110110
                pc += 1
            elif op == "cpi":
                instr = 0b11111110
                pc += 1
            elif op == "rlc":
                instr = 0b00000111
                pc += 1
            elif op == "rrc":
                instr = 0b00001111
                pc += 1
            elif op == "ral":
                instr = 0b00010111
                pc += 1
            elif op == "rar":
                instr = 0b00011111
                pc += 1
            elif op == "cma":
                instr = 0b00101111
                pc += 1
            elif op == "cmc":
                instr = 0b00111111
                pc += 1
            elif op == "stc":
                instr = 0b00110111
                pc += 1
            elif op == "ret":
                instr = 0b11001001
                pc += 1
            elif op == "pchl":
                instr = 0b11101001
                pc += 1
            elif op == "xthl":
                instr = 0b11100011
                pc += 1
            elif op == "sphl":
                instr = 0b11111001
                pc += 1
            elif op == "ei":
                instr = 0b11111011
                pc += 1
            elif op == "di":
                instr = 0b11110011
                pc += 1
            elif op == "hlt":
                instr = 0b01110110
                pc += 1
            elif op == "nop":
                instr = 0b00000000
                pc += 1
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
                    pc += 1
                elif op == "stax":
                    instr = 0b00 << 6 | rp << 4 | 0b0010
                    pc += 1
                elif op == "inx":
                    instr = 0b00 << 6 | rp << 4 | 0b0011
                    pc += 1
                elif op == "dcx":
                    instr = 0b00 << 6 | rp << 4 | 0b1011
                    pc += 1
                elif op == "dad":
                    instr = 0b00 << 6 | rp << 4 | 0b1001
                    pc += 1
                elif op == "push":
                    instr = 0b11 << 6 | rp << 4 | 0b0101
                    pc += 1
                elif op == "pop":
                    instr = 0b11 << 6 | rp << 4 | 0b0001
                    pc += 1
                elif op == "lxi":
                    imm = int(args[2])
                    instr = 0b00 << 14 | rp << 12 | 0b0001 << 8 | (imm & 0xFF) | ((imm >> 8) & 0xFF)
                    pc += 3
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
                    pc += 1
                elif op == "mvi":
                    imm = int(args[2])
                    instr = 0b00 << 14 | dest << 11 | 0b110 << 8 | (imm & 0xFF)
                    pc += 2
                elif op == "inr":
                    instr = 0b00 << 6 | dest << 3 | 0b100
                    pc += 1
                elif op == "dcr":
                    instr = 0b00 << 6 | dest << 3 | 0b101
                    pc += 1
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
                    pc += 1
                elif op == "adc":
                    instr = 0b10001 << 3 | src
                    pc += 1
                elif op == "sub":
                    instr = 0b10010 << 3 | src
                    pc += 1
                elif op == "sbb":
                    instr = 0b10011 << 3 | src
                    pc += 1
                elif op == "ana":
                    instr = 0b10100 << 3 | src
                    pc += 1
                elif op == "ora":
                    instr = 0b10110 << 3 | src
                    pc += 1
                elif op == "xra":
                    instr = 0b10101 << 3 | src
                    pc += 1
                elif op == "cmp":
                    instr = 0b10111 << 3 | src
                    pc += 1
            elif op in db_ins:
                db = int(args[1]) & 0xFF
                if op == "adi":
                    instr = 0b11000110 << 8 | db
                    pc += 2
                elif op == "aci":
                    instr = 0b11001110 << 8 | db
                    pc += 2
                elif op == "sui":
                    instr = 0b11010110 << 8 | db
                    pc += 2
                elif op == "sbi":
                    instr = 0b11011110 << 8 | db
                    pc += 2
                elif op == "ani":
                    instr = 0b11100110 << 8 | db
                    pc += 2
                elif op == "xri":
                    instr = 0b11101110 << 8 | db
                    pc += 2
                elif op == "in":
                    instr = 0b11011011 << 8 | db
                    pc += 2
                elif op == "out":
                    instr = 0b11010011 << 8 | db
                    pc += 2
            elif op in lbhb_ins:
                if not args[1].isdigit():
                    if args[1] in labels:
                        db = int(labels[args[1]]) & 0xFF
                    else:
                        labels[args[1]] = None
                        labels_to_update[args[1]] = line_counter
                        db = 0
                else:
                    lb = int(args[1]) & 0xFF
                    hb = (int(args[1]) >> 8) & 0xFF
                    db = lb << 8 | hb
                if op == "lda":
                    instr = 0b00111010 << 16 | db
                    pc += 3
                elif op == "sta":
                    instr = 0b00110010 << 16 | db
                    pc += 3
                elif op == "lhld":
                    instr = 0b00101010 << 16 | db
                    pc += 3
                elif op == "shld":
                    instr = 0b001000010 << 16 | db
                    pc += 3
                elif op == "jmp":
                    instr = 0b11000011 << 16 | db
                    pc += 3
                elif op == "jnz":
                    instr = 0b11000010 << 16 | db
                    pc += 3
                elif op == "jz":
                    instr = 0b11001010 << 16 | db
                    pc += 3
                elif op == "jnc":
                    instr = 0b11010010 << 16 | db
                    pc += 3
                elif op == "jc":
                    instr = 0b11011010 << 16 | db
                    pc += 3
                elif op == "jpo":
                    instr = 0b11100010 << 16 | db
                    pc += 3
                elif op == "jpe":
                    instr = 0b11101010 << 16 | db
                    pc += 3
                elif op == "jp":
                    instr = 0b11110010 << 16 | db
                    pc += 3
                elif op == "jm":
                    instr = 0b11111010 << 16 | db
                    pc += 3
                elif op == "call":
                    instr = 0b11001101 << 16 | db
                    pc += 3
                elif op == "cnz":
                    instr = 0b11000100 << 16 | db
                    pc += 3
                elif op == "cz":
                    instr = 0b11001100 << 16 | db
                    pc += 3
                elif op == "cnc":
                    instr = 0b11010100 << 16 | db
                    pc += 3
                elif op == "cc":
                    instr = 0b11011100 << 16 | db
                    pc += 3
                elif op == "cpo":
                    instr = 0b11100100 << 16 | db
                    pc += 3
                elif op == "cpe":
                    instr = 0b11101100 << 16 | db
                    pc += 3
                elif op == "cp":
                    instr = 0b11110100 << 16 | db
                    pc += 3
                elif op == "cm":
                    instr = 0b11111100 << 16 | db
                    pc += 3
            elif op in ret_ins:
                if op == "rnz":
                    instr = 0b11000000
                    pc += 1
                elif op == "rz":
                    instr = 0b11001000
                    pc += 1
                elif op == "rnc":
                    instr = 0b11010000
                    pc += 1
                elif op == "rc":
                    instr = 0b11011000
                    pc += 1
                elif op == "rpo":
                    instr = 0b11100000
                    pc += 1
                elif op == "rpe":
                    instr = 0b11101000
                    pc += 1
                elif op == "rp":
                    instr = 0b11110000
                    pc += 1
                elif op == "rm":
                    instr = 0b11111000
                    pc += 1
            elif op == "rst":
                n = int(args[1]) & 0xF
                instr = 0b11 << 6 | n << 3 | 0b111
                pc += 1
            # else:
            #     print("Unknown instruction:", op)
            #     continue
            if op in one_byte:
                # instructions.append("%0.2x    // %s" % (instr, line[:-1]))
                instructions.append("%0.2x" % (instr))
            elif op in two_byte:
                instructions.append("%0.4x" % (instr))
            elif op in three_byte:
                instructions.append("%0.6x" % (instr))
            line_counter += 1

else:
    print("Bad subcommand option:", subcommand)

for x in instructions:
    print(x)