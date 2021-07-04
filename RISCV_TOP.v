module RISCV_TOP (
 //General Signals
 input wire CLK,
 input wire RSTn,
 //I-Memory Signals
 output wire I_MEM_CSN,
 input wire [31:0] I_MEM_DI,//input from IM
 output reg [11:0] I_MEM_ADDR,//in byte address
 //D-Memory Signals
 output wire D_MEM_CSN,
 input wire [31:0] D_MEM_DI,
 output wire [31:0] D_MEM_DOUT,
 output wire [11:0] D_MEM_ADDR,//in word address
 output wire D_MEM_WEN,
 output wire [3:0] D_MEM_BE,
 //RegFile Signals
 output wire RF_WE,
 output wire [4:0] RF_RA1,
 output wire [4:0] RF_RA2,
 output wire [4:0] RF_WA1,
 input wire [31:0] RF_RD1,
 input wire [31:0] RF_RD2,
 output wire [31:0] RF_WD,
 output wire HALT,                   // if set, terminate program
 output reg [31:0] NUM_INST,         // number of instruction completed
 output wire [31:0] OUTPUT_PORT      // equal RF_WD this port is used for test
 );
    
/* a cache array with 8 cache lines and 1 cache line accomodating 4 words,valid bit and a 7 bit tag*/
reg [135:0] cache [7:0];
integer j;
reg cache_miss;
reg [2:0] index;
reg cache_hit;
reg [31:0] cache_miss_cycle;
wire cache_stall;
reg [31:0] num_of_hits,num_of_miss;
reg [31:0] load_from_cache;
reg [31:0] save_EX_MEM_MemRead,save_EX_MEM_MemWrite,save_EX_MEM_AluResult; // save this results so that at the final stage of the cache miss, the pipeline registers are correct
initial begin
   num_of_hits = 0;
   num_of_miss = 0;
end
assign cache_stall =  (  (cache_miss === 1'b1) && (cache_miss_cycle <= 5)  );

/*****initialize cache with valid bit 0*********/
initial begin
   for (j = 0;j<8;j = j+ 1)begin
      cache[j][128] = 1'b0;
    end
end
 ///////////////
 //
 reg [31:0] final;
 assign OUTPUT_PORT = final;
 parameter NOP = 32'h00000013;
 reg [2:0] funct3;
 reg [31:0] PC_32,IR;
 reg [6:0] funct7,opcode;
 wire isBranch,RegWrite,MemWrite,MemRead,UseImm;
 wire [1:0] IsjorB,DataRegWrite,DataRegWrite2;
 wire  [2:0] LoadExtend,Immextend;
 reg [31:0] input_one,input_two;
 wire [31:0] Alu_result; // can result be negative?? check this ****
 wire bcond;
 /* define necessary registers and wires and initialize the control,Alu and forwardandstall units */
  // IF_ID pipeline registers
 reg [31:0] IF_ID_IR;
 reg [31:0] IF_ID_cur_PC;
 reg [31:0] IF_ID_nex_PC;
 reg  IF_ID_is_NOP;
 reg IF_ID_isBranch,IF_ID_RegWrite,IF_ID_MemWrite,IF_ID_MemRead,IF_ID_UseImm;
    reg [1:0] IF_ID_IsjorB,IF_ID_DataRegWrite,IF_ID_DataRegWrite2;
    reg [2:0] IF_ID_LoadExtend,IF_ID_Immextend;



  // ID_EX pipeline registers
    reg [31:0] ID_EX_IR;
 reg [31:0] ID_EX_cur_PC;
 reg [31:0] ID_EX_nex_PC;
 reg  ID_EX_is_NOP;
 reg ID_EX_isBranch,ID_EX_RegWrite,ID_EX_MemWrite,ID_EX_MemRead,ID_EX_UseImm;
    reg [1:0] ID_EX_IsjorB,ID_EX_DataRegWrite,ID_EX_DataRegWrite2;
   reg [2:0] ID_EX_LoadExtend,ID_EX_Immextend;
 //additional control signals for Execute stage
   reg [1:0] ID_EX_AluSrc_A ,ID_EX_AluSrc_B;//from bypasssing
 reg [31:0] ID_EX_SrcA,ID_EX_SrcB; // these are the two inputs to alu if no forwarding
 reg [31:0] ID_EX_Src2;// we need to pass these to pipeline because store needs this value to write to memory



 //EX_MEM pipeline registers
 reg [31:0] EX_MEM_IR;
   reg [31:0] EX_MEM_cur_PC; // the current PC for the instryuvtion, to use pc+4 JAL,JALR
 reg  EX_MEM_is_NOP;
 reg EX_MEM_RegWrite,EX_MEM_MemWrite,EX_MEM_MemRead;
    reg [1:0] EX_MEM_DataRegWrite,EX_MEM_DataRegWrite2;
 reg [31:0] EX_MEM_ALUres;
    reg EX_MEM_bcond;
 reg [31:0] EX_MEM_Src2; // FOR STORE MEMORY USE




 //MEM_WB pipeline registers
 reg [31:0] MEM_WB_IR; 
    reg [31:0] MEM_WB_cur_PC; // the current PC for the instryuvtion, to use pc+4 JAL,JALR
 reg  MEM_WB_is_NOP;
 reg MEM_WB_RegWrite;
    reg [1:0] MEM_WB_DataRegWrite,MEM_WB_DataRegWrite2;
 reg [31:0] MEM_WB_ALUres;
    reg MEM_WB_bcond;
 reg [31:0] MEM_WB_DATA_MEM_Res; 
 //temporary registers heading to decoding stage
 reg [31:0] temp_RF_RD1,temp_RF_RD2,SignextendedImm;
 //some assign statements
 assign I_MEM_CSN = ~RSTn; // works when reset is low
   assign D_MEM_CSN = ~RSTn | ~(EX_MEM_MemRead | EX_MEM_MemWrite); // D-mem works only wheen either control signals are high otherwise i dont need to use
    assign D_MEM_WEN = ~EX_MEM_MemWrite;// Write enable negative
    assign RF_WE = MEM_WB_RegWrite; // Write enable is regwrite
    assign D_MEM_DOUT = EX_MEM_Src2; // the data written to memory is the data read from the second register
 assign D_MEM_ADDR = EX_MEM_ALUres[11:0];
 reg [3:0] Enable = 4'b1111;
 assign D_MEM_BE = Enable;
 //reg [31:0] write_data_reg;
 wire [31:0] WB_and_Forward;
 assign RF_WD = (MEM_WB_DataRegWrite == 0) ? MEM_WB_cur_PC + 4 : WB_and_Forward ;
 ///for the registers
 assign RF_RA1 = IF_ID_IR[19:15];
 assign RF_RA2 = IF_ID_IR[24:20];
 assign RF_WA1 = MEM_WB_IR[11:7];
//##############################################################################
 //wire for flush detection 
 wire flush;
 wire B_flush_T;
 wire B_flush_NT;
 wire JAL_flush;
 wire JALR_flush;

 assign B_flush_T = bcond && (     ({{20{ID_EX_IR[31]}},ID_EX_IR[7],ID_EX_IR[30:25],ID_EX_IR[11:8],1'b0}+ ID_EX_cur_PC) !== ID_EX_nex_PC    ) ; 
 assign B_flush_NT =  ~bcond && ( (ID_EX_cur_PC+4)!= ID_EX_nex_PC );
 assign JAL_flush = (ID_EX_IR[6:0] == 7'b1101111 )  &&   ( ({ {12{ID_EX_IR[31]}},ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0 } + ID_EX_cur_PC) !== ID_EX_nex_PC);
 assign JALR_flush = (ID_EX_IR[6:0] == 7'b1100111) &&  ( ({ {12{ID_EX_IR[31]}},ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0 } + ID_EX_SrcA) !== ID_EX_nex_PC);
 assign flush = (ID_EX_isBranch && (B_flush_NT || B_flush_T)) || JAL_flush || JALR_flush;
 //please dont foget this is wire from the write back stage for forwarding 
 assign WB_and_Forward = (MEM_WB_DataRegWrite2==2'b0) ? MEM_WB_ALUres : MEM_WB_DATA_MEM_Res;
 //##############################################################################


 //
  ALU Alu_unit (.input_one(input_one),.input_two(input_two),.opcode(ID_EX_IR[6:0]),.funct7(ID_EX_IR[31:25]),.funct3(ID_EX_IR[14:12]),.Alu_result(Alu_result),.bcond(bcond) );
 Control control_unit (.Inst(IR),.IsjorB(IsjorB),.DataRegWrite(DataRegWrite),.isBranch(isBranch),.RegWrite(RegWrite),.MemWrite(MemWrite),.MemRead(MemRead),.UseImm(UseImm),.DataRegWrite2(DataRegWrite2),.LoadExtend(LoadExtend),.Immextend(Immextend) );
 wire stall;
 wire [1:0] AluSrc_A,AluSrc_B;
 forwardAndStall foward_unit(.IF_ID(IF_ID_IR) ,.ID_EX(ID_EX_IR),.EX_MEM(EX_MEM_IR) ,.stall_flag(stall) ,.AluSrc_A(AluSrc_A) ,.AluSrc_B(AluSrc_B));
 initial begin
  NUM_INST <= 0;
 end
 // HALT
   assign HALT = (EX_MEM_IR === 32'h00c00093 && ID_EX_IR === 32'h00008067);
 // HALT
 // Only allow for NUM_INST

 
 always @ (negedge CLK) begin // modify this, instruction should be executed 
  if (RSTn && (MEM_WB_IR !== 32'bx)) begin
    if (MEM_WB_is_NOP || (MEM_WB_IR  != 32'h00000013)) begin
      if (cache_stall) begin
         if (cache_miss_cycle == 32'b1)
            NUM_INST <= NUM_INST + 1;
      end
      else if (cache_miss_cycle  !== 6)
         NUM_INST <= NUM_INST + 1;
     end
  end
 end


     
 always @(RSTn) begin
     if (RSTn)
       PC_32 = 32'b0;
  end
 always @(PC_32) begin // WHEN PC_32 changes, fetch an instruction
  I_MEM_ADDR = PC_32[11:0];
 end
 // if stall !== 1 latch the pipeline registers and next_pc (ALWAYS NOT TAKEN FOR NOW, THEN UPDATE TO BTB)
 always @(I_MEM_DI)begin
  IR = I_MEM_DI;
  //opcode = IR[6:0]; // giving the opcode
     //funct7 = IR[31:25];  // if there is no funct7 
     //funct3 = IR[14:12];
  // continue 
 end

  //fetching stage you shouldnt modify IF_ID except this stage
  always @(posedge CLK)begin
      if ((cache_stall === 1'b0) || (cache_stall === 1'bx)) begin
         if(stall!==1 && flush!==1)begin
      //latching the controll signals generated and the instruction
            IF_ID_IR <= IR;
            IF_ID_cur_PC<=PC_32;
            IF_ID_nex_PC<= PC_32 + 4;
            IF_ID_is_NOP <= (IR==32'h00000013);
            IF_ID_isBranch <= isBranch;
            IF_ID_RegWrite <=RegWrite;
            IF_ID_MemWrite <=MemWrite;
            IF_ID_MemRead <= MemRead;
            IF_ID_UseImm <= UseImm;
               IF_ID_IsjorB <= IsjorB;
            IF_ID_DataRegWrite <=DataRegWrite;
            IF_ID_DataRegWrite2 <=DataRegWrite2;
                  IF_ID_LoadExtend <=LoadExtend;
            IF_ID_Immextend <=Immextend;
            //pc prediction (use btb when you implement it)
            PC_32 <= PC_32 +4;
         end
               else if(flush ===1)begin
            IF_ID_IR <= NOP;
            IF_ID_cur_PC<=PC_32;
            IF_ID_nex_PC<= PC_32 + 4;
            IF_ID_is_NOP <= 0;
            IF_ID_isBranch <= 0;
            IF_ID_RegWrite <=1;
            IF_ID_MemWrite <=0;
            IF_ID_MemRead <= 0;
            IF_ID_UseImm <= 1;
               IF_ID_IsjorB <= 0;
            IF_ID_DataRegWrite <=2;
            IF_ID_DataRegWrite2 <=0;
                  IF_ID_LoadExtend <=2'bx;
            IF_ID_Immextend <=2'bx;
            //pc prediction (use btb when you implement it)
            if (ID_EX_isBranch && B_flush_NT)
               PC_32 <= PC_32 +4;
            else if (ID_EX_isBranch && B_flush_T)
               PC_32 <= {{20{ID_EX_IR[31]}},ID_EX_IR[7],ID_EX_IR[30:25],ID_EX_IR[11:8],1'b0} + ID_EX_cur_PC;
            else if (ID_EX_IR[6:0] == 7'b1101111) //JAL
               PC_32 <= { {12{ID_EX_IR[31]}},ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0 } + ID_EX_cur_PC;
            else 
               PC_32 <= { {12{ID_EX_IR[31]}},ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0 } + ID_EX_SrcA;
         end
      end   
   end
 
 always @(RF_RD1,RF_RD2)begin
   temp_RF_RD1= RF_RD1;
   temp_RF_RD2= RF_RD2;   
  end
//For  sign extended immediate
 always @(IF_ID_IR) begin
 if (IF_ID_IR[6:0] == 7'b1101111 || IF_ID_IR[6:0] == 7'b1100111) // JAL,JALR
  SignextendedImm =  { {12{IF_ID_IR[31]}},IF_ID_IR[19:12],IF_ID_IR[20],IF_ID_IR[30:21],1'b0 };
 else if (IF_ID_IR[6:0] == 7'b1100011) // branch
  SignextendedImm = { {20{IF_ID_IR[31]}},IF_ID_IR[7],IF_ID_IR[30:25],IF_ID_IR[11:8],1'b0 };
 else if ( IF_ID_IR[6:0] == 7'b0010011 || IF_ID_IR[6:0] == 7'b0000011) //ALUI or load
  SignextendedImm = { {20{IF_ID_IR[31]}},IF_ID_IR[31:20] };
 else
  SignextendedImm = { {20{IF_ID_IR[31]}},IF_ID_IR[31:25],IF_ID_IR[11:7] };
    end


  //decoding stage
  always @(posedge CLK)begin
   //if it isnt stalling then latch it
      if ((cache_stall === 1'b0) || (cache_stall === 1'bx)) begin
            if(stall!== 1 && flush!==1)begin
            //**************************************check for flush signal
               ID_EX_IR <= IF_ID_IR;
            ID_EX_cur_PC<=IF_ID_cur_PC;
            ID_EX_nex_PC<=IF_ID_nex_PC;
            ID_EX_is_NOP <= IF_ID_is_NOP;
            ID_EX_isBranch <= IF_ID_isBranch;
            ID_EX_RegWrite<= IF_ID_RegWrite;
            ID_EX_MemWrite<= IF_ID_MemWrite;
            ID_EX_MemRead<= IF_ID_MemRead;
            ID_EX_UseImm<= IF_ID_UseImm;
               ID_EX_IsjorB<= IF_ID_IsjorB;
            ID_EX_DataRegWrite<= IF_ID_DataRegWrite;
            ID_EX_DataRegWrite2<= IF_ID_DataRegWrite2;
               ID_EX_LoadExtend<= IF_ID_LoadExtend;
            ID_EX_Immextend<= IF_ID_Immextend;
            //additional control signals for Execute stage
               ID_EX_AluSrc_A <= AluSrc_A;
            ID_EX_AluSrc_B <=AluSrc_B;//from bypasssing
            ID_EX_SrcA<= temp_RF_RD1; 
            //////////the mux//////////////////////////
            if(IF_ID_UseImm)
               ID_EX_SrcB <= SignextendedImm;
            else
               ID_EX_SrcB <= temp_RF_RD2;
            ///////////////////////////////////////////// 
            ID_EX_Src2 <= temp_RF_RD2;// we need to pass these to pipeline because store needs this value to write to memory
            end
            //forward the nop instruction with it's controll signal
            else
            begin
               ID_EX_IR <= NOP;
               ID_EX_cur_PC<=IF_ID_cur_PC;
               ID_EX_nex_PC<=IF_ID_nex_PC;
               ID_EX_is_NOP <= 0;
               ID_EX_isBranch <= 0;
               ID_EX_RegWrite<= 1;
               ID_EX_MemWrite<= 0;
               ID_EX_MemRead<= 0;
               ID_EX_UseImm<= 1;
               ID_EX_IsjorB<= 0;
               ID_EX_DataRegWrite<=2;
               ID_EX_DataRegWrite2<= 0;
               ID_EX_LoadExtend<= 2'bx;
               ID_EX_Immextend<= 2'bx;
               //additional control signals for Execute stage
               ID_EX_AluSrc_A <=0;
               ID_EX_AluSrc_B <=0;//from bypasssing
               ID_EX_SrcA<= 0; 
               ID_EX_SrcB <= 0;
               ///////////////////////////////////////////// 
               ID_EX_Src2 <= 0;// we need to pass these to pipeline because store needs this value to write to memory
            end 
      end
  end

//setting up the ALU
always @(ID_EX_IR)begin
 //donot follow the picture
 if(ID_EX_AluSrc_A==0)
  input_one = ID_EX_SrcA;
 else if(ID_EX_AluSrc_A == 1)
  input_one = WB_and_Forward;
 else if(ID_EX_AluSrc_A==2)
  input_one = EX_MEM_ALUres;

//trust the controll signal
 if(ID_EX_AluSrc_B==0)
  input_two = ID_EX_SrcB;
 else if(ID_EX_AluSrc_B== 1)
  input_two = WB_and_Forward;
 else if(ID_EX_AluSrc_B==2)
  input_two = EX_MEM_ALUres;

end



  //Execute stage
  always @(posedge CLK)begin
      if ((cache_stall === 1'b0) || (cache_stall === 1'bx)) begin
            EX_MEM_IR <= ID_EX_IR;
            EX_MEM_cur_PC<= ID_EX_cur_PC; // the current PC for the instryuvtion, to use pc+4 JAL,JALR
         EX_MEM_is_NOP<= ID_EX_is_NOP;
         EX_MEM_RegWrite<= ID_EX_RegWrite;
         EX_MEM_MemWrite<= ID_EX_MemWrite;
         EX_MEM_MemRead<= ID_EX_MemRead;
            EX_MEM_DataRegWrite<= ID_EX_DataRegWrite;
         EX_MEM_DataRegWrite2<= ID_EX_DataRegWrite2;
         EX_MEM_ALUres<= Alu_result;
            EX_MEM_bcond <= bcond;
         if (ID_EX_IR[6:0] === 7'b0100011) begin // store
         if( (ID_EX_IR[24:20] !== EX_MEM_IR[11:7]) && (ID_EX_IR[24:20] !== MEM_WB_IR[11:7]) ) // second source register has no dependency
            EX_MEM_Src2 <= ID_EX_Src2;
         else if(  (ID_EX_IR[24:20] === MEM_WB_IR[11:7]) )
            EX_MEM_Src2 <= WB_and_Forward;
         else if( (ID_EX_IR[24:20] === EX_MEM_IR[11:7]) )
            EX_MEM_Src2 <= EX_MEM_ALUres; 
         end 
         else begin
         if(ID_EX_AluSrc_B==0)
            EX_MEM_Src2 <= ID_EX_Src2;
         else if(ID_EX_AluSrc_B== 1)
            EX_MEM_Src2 <= WB_and_Forward;
         else if(ID_EX_AluSrc_B==2)
            EX_MEM_Src2 <= EX_MEM_ALUres; 
         end


 // FOR STORE MEMORY USE
 // stall , apppend nop
    //latch
    // request from register
    // set up the mux based on the controll signal
    end
  end
  //write bakc
  always@(posedge CLK)begin
      if ((cache_stall === 1'b0) || (cache_stall === 1'bx)) begin
         MEM_WB_IR <= EX_MEM_IR; 
            MEM_WB_cur_PC<= EX_MEM_cur_PC; // the current PC for the instryuvtion, to use pc+4 JAL,JALR
         MEM_WB_is_NOP<= EX_MEM_is_NOP;
         MEM_WB_RegWrite<= EX_MEM_RegWrite;
            MEM_WB_DataRegWrite<= EX_MEM_DataRegWrite;
         MEM_WB_DataRegWrite2<= EX_MEM_DataRegWrite2;
         MEM_WB_ALUres<= EX_MEM_ALUres;
            MEM_WB_bcond<= EX_MEM_bcond;
         if (cache_hit && (EX_MEM_IR === 7'b0000011)) // if cache hit and it is a load don't take value from D_MEM_DI
            MEM_WB_DATA_MEM_Res <= load_from_cache;
         else if (cache_miss && (cache_miss_cycle == 6) && (EX_MEM_IR[6:0] === 7'b0000011))
            MEM_WB_DATA_MEM_Res <= load_from_cache;
         else
            MEM_WB_DATA_MEM_Res<=D_MEM_DI; 
      end
  end

  always @(RF_WD,MEM_WB_IR) begin
   if (MEM_WB_IR[6:0] === 7'b0100011 ) // store
    final = {20'b0, MEM_WB_ALUres[11:0]};
   else if (MEM_WB_IR[6:0] === 7'b1100011)
    final = MEM_WB_bcond;
   else
    final = RF_WD;
  end



/*********** cache part********/



reg [13:0] cache_pointer;
reg  [6:0] cache_tag; // for debugging
reg  [6:0] pointer_tag; // for debugging
always @(EX_MEM_IR) begin // whenever EX_MEM instruction changes
   cache_pointer = EX_MEM_ALUres[13:0];
   index = cache_pointer[6:4];
   if ( EX_MEM_IR[6:0] === 7'b0100011 || EX_MEM_IR[6:0] === 7'b0000011 ) begin // if it is a load or store
      if ( (cache[index][135:129] === cache_pointer[13:7]) && (cache[index][128] === 1) ) begin // cache hit
         num_of_hits = num_of_hits + 1;
         cache_miss = 0;
         cache_hit = 1;
         // do appropriate thing
         // if it is aload instruction
         if (EX_MEM_IR === 7'b0000011)// if it is a load and a hit don't read from memory, at the negative edge read from cache
            EX_MEM_MemRead = 0;
      end
      else begin
         /***saved values*****/
        save_EX_MEM_MemRead = EX_MEM_MemRead;
        save_EX_MEM_AluResult = EX_MEM_ALUres;
        save_EX_MEM_MemWrite = EX_MEM_MemWrite;
        // do appropriate thing
         num_of_miss = num_of_miss + 1;
        cache_miss = 1;
        cache_hit = 0;
        cache_miss_cycle = 1;
        
      end
   end
   else begin
      cache_miss = 0;
      cache_hit = 0;
   end
end

/** for debugging**/
always @(EX_MEM_IR) begin
  if ( EX_MEM_IR[6:0] === 7'b0100011 || EX_MEM_IR[6:0] === 7'b0000011 ) begin
      cache_tag = cache[index][135:129];
      pointer_tag = cache_pointer[13:7];
  end
  else begin
      cache_tag = 7'bx;
      pointer_tag = 7'bx;
   end
      
end

/****** handle cache hit**********/
always @(negedge CLK)begin // if it is a cache_hit
   if (cache_hit && (EX_MEM_IR[6:0] === 7'b0100011)) begin // if cache hit and it is a store, WRITE SYNCHRONOUSLY to cache, mem model handles the write to mem
         if (cache_pointer[3:2] == 2'b00) // if block offset is 0 -- update the first word
            cache[index][31:0] <= D_MEM_DOUT;
         else if (cache_pointer[3:2] == 2'b01) // update the second word
            cache[index][63:32] <= D_MEM_DOUT;
         else if (cache_pointer[3:2] == 2'b10) //update the third word
            cache[index][95:64] <= D_MEM_DOUT;
         else if (cache_pointer[3:2] == 2'b11) //update the fourth word
            cache[index][127:96] <= D_MEM_DOUT;
   end
   else if (cache_hit && (EX_MEM_IR[6:0] === 7'b0000011)) begin // if cache hit , just load from cache, don't access mem :)
      if (cache_pointer[3:2] == 2'b00) // if block offset is 0 -- load the first word
            load_from_cache <=cache[index][31:0];
      else if (cache_pointer[3:2] == 2'b01) // load the second word
            load_from_cache <= cache[index][63:32];
      else if (cache_pointer[3:2] == 2'b10) //load the third word
           load_from_cache <= cache[index][95:64];
      else if (cache_pointer[3:2] == 2'b11) //load the fourth word
            load_from_cache <= cache[index][127:96]; 
   end

   if (cache_miss && (cache_miss_cycle == 6) && (EX_MEM_IR[6:0] === 7'b0100011)) begin // store and last cycle, update the cache synchronously as mem model updates mem
     if (cache_pointer[3:2] == 2'b00) // if block offset is 0 -- update the first word
         cache[index][31:0] <= D_MEM_DOUT;
     else if (cache_pointer[3:2] == 2'b01) // update the second word
         cache[index][63:32] <= D_MEM_DOUT;
     else if (cache_pointer[3:2] == 2'b10) //update the third word
         cache[index][95:64] <= D_MEM_DOUT;
     else if (cache_pointer[3:2] == 2'b11) //update the fourth word
         cache[index][127:96] <= D_MEM_DOUT;

   end

   else if (cache_miss && (cache_miss_cycle == 6) && (EX_MEM_IR[6:0] === 7'b0000011)) begin // load and last cycle, just load from cache ;))))
      if (cache_pointer[3:2] == 2'b00) // if block offset is 0 -- load the first word
            load_from_cache <=cache[index][31:0]; 
      else if (cache_pointer[3:2] == 2'b01) // load the second word
            load_from_cache <= cache[index][63:32]; 
      else if (cache_pointer[3:2] == 2'b10) //load the third word
           load_from_cache <= cache[index][95:64]; 
      else if (cache_pointer[3:2] == 2'b11) //load the fourth word
            load_from_cache <= cache[index][127:96]; 
   end

end



always @(posedge CLK) begin  // update the cache_miss_cycle
   if (cache_miss_cycle !== 32'bx)
      cache_miss_cycle <= cache_miss_cycle + 1;
end

/********** depending on the cycle make the appropriate control signals for loading to cache    *****/
always @(cache_miss_cycle,cache_miss) begin 
  if (cache_miss && (cache_miss_cycle == 1))begin
    //next stage you will update cache so update tag and valid bit
    cache[index][135:129] = cache_pointer[13:7];
    cache[index][128] = 1'b1;
    // no memory access in this stage, silent stage, do nothing
    EX_MEM_MemRead = 0;
    EX_MEM_MemWrite = 0;


  end
  else if (cache_miss && (cache_miss_cycle == 2))begin
    
    EX_MEM_MemWrite = 0;
    EX_MEM_MemRead = 1; //READ FROM MEMORY FOR THE FIRST LOAD in
    EX_MEM_ALUres[3:2] = 2'b00; // get ready for the first load


  end

  else if (cache_miss && (cache_miss_cycle == 3))begin
    EX_MEM_MemWrite = 0;
    EX_MEM_MemRead = 1; //READ FROM MEMORY FOR THE SECOND LOAD in
    EX_MEM_ALUres[3:2] = 2'b01; // get ready for the second load

  end
  else if (cache_miss && (cache_miss_cycle == 4))begin
    EX_MEM_MemWrite = 0;
    EX_MEM_MemRead = 1; //READ FROM MEMORY FOR THE SECOND LOAD in
    EX_MEM_ALUres[3:2] = 2'b10; // get ready for the second load

  end
  else if (cache_miss && (cache_miss_cycle == 5))begin
    EX_MEM_MemWrite = 0;
    EX_MEM_MemRead = 1; //READ FROM MEMORY FOR THE SECOND LOAD in
    EX_MEM_ALUres[3:2] = 2'b11; // get ready for the second load

  end
  else if (cache_miss && (cache_miss_cycle == 6))begin
    EX_MEM_MemWrite = save_EX_MEM_MemWrite;
    EX_MEM_MemRead = save_EX_MEM_MemRead;
    EX_MEM_ALUres = save_EX_MEM_AluResult;

  end



end

/*** write the load values at the end of the cycle*******/

always @(posedge CLK) begin
  
  if (cache_miss && (cache_miss_cycle == 2))begin
    cache[index][31:0] <= D_MEM_DI; // write the first word loaded from memory

  end

  else if (cache_miss && (cache_miss_cycle == 3))begin
     cache[index][63:32] <= D_MEM_DI; // write the second word loaded from memory

  end
  else if (cache_miss && (cache_miss_cycle == 4))begin
    cache[index][95:64] <= D_MEM_DI;

  end

  else if (cache_miss && (cache_miss_cycle == 5))begin
    cache[index][127:96] <= D_MEM_DI;

  end
  

end
 

endmodule







