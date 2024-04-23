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

    //enable stage registers
    reg f0=1;
    reg f1=0;
    reg d0=0;
    reg e0=0;
    reg e1=0;
    reg wb=0;

    // PC
    reg [15:0]pc = 16'h0000;
    reg [15:0]pc2;
    reg [15:0]pc3;
    reg [15:0]pc4;
    reg [15:0]pc5;
    reg [15:0]pc6;

    // read from memory
    wire[15:0] ins;
    wire [15:0] instruction;
    wire [15:0]mem_raddr1;
    wire[15:0] mem_load;
    wire mem_wen;
    wire [15:0]mem_waddr;
    wire [15:0]mem_wdata;

    wire [15:0]real_pc=(load_load || misaligned_load || misaligned_store) ? pc2 : pc;


    reg [19:0] predictor[0:16'h07ff];


    // memory
    mem memory(clk,real_pc[15:1],ins,mem_raddr1[15:1],mem_load,mem_wen,mem_waddr[15:1],mem_wdata[15:0]);


    wire [3:0]reg_raddr0;
    wire[15:0]ra_init;
    wire [3:0]reg_raddr1;
    wire[15:0]rt_init;
    wire reg_wen;
    wire [3:0]reg_waddr;
    wire [15:0]reg_wdata;
    // registers
    regs registers(clk,reg_raddr0[3:0],ra_init[15:0],reg_raddr1[3:0],rt_init[15:0],reg_wen,reg_waddr[3:0],reg_wdata[15:0]);

    assign instruction= ((justStalled || justMisalignLoadStalled || justMisalignStoreStalled || justStalled2 ||justMisalignLoadStalled2 || justMisalignStoreStalled2) && oddPc) ?  {prev_instruction[7:0],prev_instruction2[15:8]} :
                        (justStalled && justMisalignLoadStalled2) ? prev_instruction2 :
                        (justStalled || justMisalignLoadStalled || justMisalignStoreStalled) ? prev_instruction :
                        (!oddPc) ? ins :
                        (!prev_instruction_valid) ? {ins[7:0], mem_load[15:8]} :
                        {ins[7:0], prev_instruction[15:8]};

    // decode stage
    // figure out what instruction is running
    wire [3:0] opcode=instruction[15:12];
    wire [3:0] ra_addr=instruction[11:8];
    wire [3:0] rb_addr=instruction[7:4];
    wire [3:0] rt_addr=instruction[3:0];

    wire sub = (opcode == 4'b0000);

    wire movl = (opcode == 4'b1000);
    wire movh = (opcode == 4'b1001);

    wire equal = (opcode == 4'b1110 && rb_addr == 4'b0000);
    wire notequal = (opcode == 4'b1110 && rb_addr == 4'b0001);
    wire less = (opcode == 4'b1110 && rb_addr == 4'b0010);
    wire greaterequal = (opcode == 4'b1110 && rb_addr == 4'b0011);

    wire load = (opcode == 4'b1111 && rb_addr == 4'b0000);
    wire store = (opcode == 4'b1111 && rb_addr == 4'b0001);

    reg[15:0] prev_instruction;
    reg[15:0] prev_instruction2;
    reg prev_instruction_valid=0;

    //propogated instructions so I can use them later on
    reg[8:0] dummySignal2;
    reg[8:0] dummySignal3;
    reg[8:0] dummySignal4;
    // read results after propagation
    wire is_sub=dummySignal4[8];
    wire is_movl=dummySignal4[7];
    wire is_movh=dummySignal4[6];
    wire is_equal=dummySignal4[5];
    wire is_notequal=dummySignal4[4];
    wire is_less=dummySignal4[3];
    wire is_greaterequal=dummySignal4[2];
    wire is_load=dummySignal4[1];
    wire is_store=dummySignal4[0];

    //loading after loading hazards
    wire load_load=(d0 && ra_addr==second_register_raddr && dummySignal2[1] && e0 && load && !misaligned_load && !misaligned_load);

    wire misaligned_load = (e0 && dummySignal2[1] && ra_fake[0]==1 && !justMisalignLoadStalled) && !(e1 && dummySignal3[0] && ra_fake==ra_fake_middle);

    wire misaligned_store = (e0 && dummySignal2[0] && ra_fake[0]==1 && !justMisalignStoreStalled);

    //wire load_after_move=(d0 && ra_addr==dummy1 && (dummySignal2[8] || dummySignal2[7] || dummySignal2[6]) && e0 && load);

    //first read address for regsiters - will be propagated through dummy registers
    assign reg_raddr0=ra_addr;
    reg [3:0]dummy5;
    reg[3:0] dummy6;

    wire [3:0]first_register_raddr=dummy5;
    wire [3:0]first_register_raddr2=dummy6;


    //second read address for regsiters - will be propagated through dummy registers
    assign reg_raddr1=(sub) ? rb_addr : rt_addr;
    reg [3:0]dummy7;
    reg[3:0] dummy8;

    wire [3:0]second_register_raddr=dummy7;
    wire [3:0]second_register_raddr2=dummy8;


    //update register values based on if register was r0 or if forwarding should happen
    wire [15:0]ra_fake=(justMisalignLoadStalled || justMisalignStoreStalled) ? ra_fake_middle :
                        (first_register_raddr==0) ? 0 : 
                            (first_register_raddr==dummy2 && e1 && (dummySignal3[8] || dummySignal3[7] || dummySignal3[6])) ? reg_wdata_fake :
                                (first_register_raddr==reg_waddr && reg_wen) ? reg_wdata :ra_init;
    wire [15:0]rt_fake=(justMisalignLoadStalled || justMisalignStoreStalled) ? rt_fake_middle :
                        (second_register_raddr==0) ? 0 : 
                                (second_register_raddr==reg_waddr && reg_wen) ? reg_wdata :rt_init;


    //propagate registers into writeback stage and also see if forwarding should happen in the middle
    reg [15:0]ra_fake1;
    wire [15:0] ra_fake_middle=(first_register_raddr2==reg_waddr && reg_wen && reg_waddr!=0)? reg_wdata : ra_fake1;
    reg [15:0]ra_fake2;
    wire[15:0] ra = ra_fake2;

    reg [15:0]rt_fake1;
    wire [15:0] rt_fake_middle=(second_register_raddr2==reg_waddr && reg_wen && reg_waddr!=0)? reg_wdata : rt_fake1;
    reg [15:0]rt_fake2;
    wire[15:0] rt = rt_fake2;

    //check if instruction is valid
    wire isValid=((dummySignal3[0] || dummySignal3[1]  || dummySignal3[2]  || dummySignal3[3]  || dummySignal3[4]  || dummySignal3[5]  || dummySignal3[6]  || dummySignal3[7]  || dummySignal3[8] )||!e1)||(pc_change && pc6!=pc5);

    //calculate immediate and propagate it through
    wire [7:0]imm=instruction[11:4];
    reg[7:0] imm1;
    reg[7:0] imm2;
    reg[7:0] imm3;
    wire [7:0] immediate = imm3;

    //set write address and propagate it through
    wire[3:0] reg_waddrfake=rt_addr;
    reg[3:0] dummy1;
    reg[3:0] dummy2;
    reg[3:0] dummy3;
    
    assign reg_waddr=dummy3;

    //tell memory what to read from
    assign mem_raddr1=(justOddBranched) ? real_pc-2: 
                        (justMisalignLoadStalled || justMisalignStoreStalled) ? misalignLoadNewAddress: 
                            ra_fake;
    
    //decide what should be written into a register
    wire[15:0] reg_wdata_fake=(dummySignal3[8]) ? ra_fake_middle-rt_fake_middle :
                                (dummySignal3[7]) ? { {8{imm2[7]}}, imm2 } :
                                    (dummySignal3[6]) ? (rt_fake_middle&8'b11111111 | (imm2<<8)) : 0;
    
    reg[15:0] reg_wdata_register;
    assign reg_wdata=(is_sub) ? ra-rt :
                    (is_movl) ? { {8{immediate[7]}}, immediate } :
                    (is_movh) ? (rt&8'b11111111 | (immediate<<8)) : 
                    (justMisalignStoreStalled3) ? mem_load :
                    (ra==mem_waddrfuture && mem_wenfuture) ? mem_wdatafuture :
                    (ra[0]==1 && ra-2==mem_waddrfuture && mem_wenfuture) ? {mem_load[15:8], mem_wdatafuture[15:8]} :
                    ((justMisalignStoreStalled2 || justMisalignLoadStalled2) && ra-1==mem_waddrfuture && mem_wenfuture) ? mem_wdatafuture :
                    (ra[0]==0 && ra-1==mem_waddrfuture && mem_wenfuture) ?  {mem_load[15:8], mem_wdatafuture[15:8]}:
                    (ra[0]==1 && ra-1==mem_waddrfuture && mem_wenfuture) ?   {mem_wdatafuture[15:8], mem_load[7:0]}:
                    (justMisalignLoadStalled3) ?  {mem_load[7:0], futureLoadData[15:8]}:
                    mem_load;


    //figure out if we should branch
    wire pc_change = ((is_equal && ra==0) || (is_notequal && ra!=0) || (is_less && ra[15]==1) || (is_greaterequal && ra[15]==0)) && wb ;

    //figure out if we write to registers
    assign reg_wen=((is_sub || is_movl || is_movh || is_load) ) && wb;

    // flushing for self modifying code
    wire self_modifying=(mem_wen && (mem_waddr==pc2 || mem_waddr==pc3 || mem_waddr==pc4 || mem_waddr==pc5) );
    wire self_modifying2=(mem_wen && (mem_waddr==pc2+1 || mem_waddr==pc3+1 || mem_waddr==pc4+1 || mem_waddr==pc5+1) );
    wire self_modifying3=(mem_wen && (mem_waddr==pc2-1 || mem_waddr==pc3-1 || mem_waddr==pc4-1 || mem_waddr==pc5-1) );

    assign mem_waddr=(justMisalignStoreStalled2) ? misaligned_store_addr :
                      (justMisalignStoreStalled3) ? misaligned_store_addr2+2 :
                        ra;
    reg[15:0] mem_waddrfuture;

    //figure out what to write into memory and propagate values once for load after store forwarding
    assign mem_wen = is_store && wb;
    reg mem_wenfuture;


    assign mem_wdata=(justMisalignStoreStalled2) ? {misaligned_store_data[7:0], reg_wdata[7:0]} :
                      (justMisalignStoreStalled3) ? {reg_wdata[15:8], misaligned_store_data2[15:8]} :
                        rt;

    reg[15:0] mem_wdatafuture;

    //propagate printing details so print is delayed by a cycle
    wire print = reg_wen && reg_waddr==0 && wb && !justMisalignLoadStalled2;

    reg oddPc=0;

    reg justOddBranched=0;

    reg justStalled=0;
    reg justStalled2;

    reg justMisalignLoadStalled;
    reg justMisalignLoadStalled2;
    reg justMisalignLoadStalled3;

    reg justMisalignStoreStalled;
    reg justMisalignStoreStalled2;
    reg justMisalignStoreStalled3;

    reg[15:0] misaligned_store_data;
    reg[15:0] misaligned_store_addr;
    reg[15:0] misaligned_store_data2;
    reg[15:0] misaligned_store_addr2;


    reg[15:0] misalignLoadNewAddress=0;

    reg[15:0] futureLoadData;


    always @(posedge clk) begin
        halt<=!isValid;

        justStalled<=load_load;
        justStalled2<=justStalled;

    
        justMisalignLoadStalled<=misaligned_load;
        justMisalignLoadStalled2<=justMisalignLoadStalled;
        justMisalignLoadStalled3<=justMisalignLoadStalled2;

        justMisalignStoreStalled<=misaligned_store;
        justMisalignStoreStalled2<=justMisalignStoreStalled;
        justMisalignStoreStalled3<=justMisalignStoreStalled2;

        misaligned_store_data<=rt_fake_middle;
        misaligned_store_data2<=misaligned_store_data;

        misaligned_store_addr<=ra_fake_middle;
        misaligned_store_addr2<=misaligned_store_addr;

        futureLoadData<=reg_wdata;
        //handle all cases of flushing
        if(pc_change && pc6!=pc5) begin 
            if(rt[0]==1) begin
                pc<=rt+2;
                justOddBranched<=1;
            end
            else begin
                pc<=rt;
            end
            f1<=0;
            d0<=0;
            e0<=0;
            e1<=0;
            wb<=0;
            oddPc<=(rt[0]==1);
            prev_instruction_valid<=0;
        end
        else begin 
            if (self_modifying || self_modifying2 || self_modifying3) begin
                if(pc6[0]==1) begin
                    pc<=pc6+2;
                    justOddBranched<=1;
                end
                else begin
                    pc<=pc6+2;
                end
                f1<=0;
                d0<=0;
                e0<=0;
                e1<=0;
                wb<=0;
                oddPc<=(pc6[0]==1);
                prev_instruction_valid<=0;
            end
            else if (misaligned_load || misaligned_store) begin
                pc<=real_pc+2;
                misalignLoadNewAddress<=ra_fake+2;
                wb<=e1;

            end
            else if(load_load) begin
                pc<=real_pc+2;
                e0<=0;
                e1<=e0;
                wb<=e1;
            end
            else begin
                pc <= real_pc + 2;
                f1<=f0;
                d0<=f1;
                e0<=d0;
                e1<=e0;
                wb<=e1;
            end
            if(d0&&oddPc && !(self_modifying || self_modifying2 || self_modifying3))begin
                prev_instruction_valid<=1;
            end
        end

        if(justOddBranched==1)begin
            justOddBranched<=0;
        end

        if(!misaligned_load && !misaligned_store) begin
            dummy1<=reg_waddrfake;
            imm1<=imm;
            dummy5<=reg_raddr0;
            dummy7<=reg_raddr1;
            dummySignal2<={sub, movl, movh, equal, notequal, less, greaterequal, load, store};
            pc4<=pc3;
        end

        reg_wdata_register<=reg_wdata_fake;

        prev_instruction<=ins;
        prev_instruction2<=prev_instruction;

        //propagate values through dummy registers
        mem_waddrfuture<=ra;

        mem_wenfuture<=mem_wen;

        mem_wdatafuture<=rt;


        dummy2<=dummy1;
        dummy3<=dummy2;


        imm2<=imm1;
        imm3<=imm2;


        dummy6<=dummy5;


        dummy8<=dummy7;


        dummySignal3<=dummySignal2;
        dummySignal4<=dummySignal3;


        ra_fake1<=ra_fake;
        ra_fake2<=ra_fake_middle;

        rt_fake1<=rt_fake;
        rt_fake2<=rt_fake_middle;

        pc2<=real_pc;
        pc3<=pc2;

        pc5<=pc4;
        pc6<=pc5;


        if(print && halt==0) begin
            $write("%c",(reg_wdata&8'b11111111));
        end
    end


endmodule
