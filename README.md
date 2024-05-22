# Intel 8080 Implementation Project

## Overview

This project involves the full implementation of the Intel 8080 instruction set in Verilog. Our goal was to create a functional representation of the 8080 CPU, along with an assembler to facilitate writing 8080 code.

## Components

### Assembler
We created an assembler to simplify writing 8080 code. This assembler handles pseudo instructions, such as labels, making it more user-friendly.

### 8080 Architecture
Key differences between the architecture of our previous projects (p7 & p8) and this project include:
- **Instruction Length:** Instructions in the 8080 are 1, 2, or 3 bytes long. We read 24 bits for each instruction and determine the actual length in the decode stage. We no-op previous instructions based on the determined length.
- **Calculations in Writeback:** Calculations are performed in the writeback stage to utilize flags or loaded data, which are only set after writeback. This introduces stalling hazards for instructions like load after move, requiring additional stalling if flags need updating.
- **Extended Register Ports:** We added more register ports to accommodate the extended ISA, allowing multiple registers to be read and written simultaneously (e.g., XCHG instruction).

### Interrupts
Interrupts were abstracted to an external Verilog module, which can be reprogrammed to provide interrupt signals as needed.

### Branch Predictor
We developed a branch predictor to handle variable-length instructions in the 8080, ensuring accurate prediction of the start of the next instruction.

### Stack & Subroutines
Our memory module includes a stack for operations such as pushing, popping, and resetting the stack pointer. Subroutines are supported by pushing and popping the link register off the stack, ensuring correct return locations.

## Running the Code
The code is available in the “pa” folder and on GitHub: [8080 Project GitHub Repository](https://github.com/annie551/8080).

### Assembler
To run the assembler, use the following command:
```sh
assembler.py asm [file.asm] > [file.hex]
```

### Full Component Execution
To run all components, use:
```sh
assembler.py asm asmTest.asm > asmTest.hex
make -s asmTest.test
```

## References
- [8080 Programmer's Manual](https://altairclone.com/downloads/manuals/8080%20Programmers%20Manual.pdf)
