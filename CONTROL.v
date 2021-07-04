


module Control(Inst,IsjorB,DataRegWrite,Immextend,RegWrite,UseImm,DataRegWrite2,LoadExtend,MemWrite,MemRead,isBranch);
  reg [2:0] funct3;
  reg [6:0] funct7,opcode;
  input [31:0] Inst;
  output reg isBranch,RegWrite,MemWrite,MemRead,UseImm;
  output reg [1:0] IsjorB,DataRegWrite,DataRegWrite2;
  output reg [2:0] LoadExtend,Immextend;

  always @(*) begin
    funct3 = Inst[14:12];
    funct7 = Inst[31:25];
    opcode = Inst[6:0];
    // reset the signals
    isBranch = 1'b0;
    RegWrite = 1'b0;
    MemWrite = 1'b0;
    MemRead = 1'b0;
    UseImm = 1'b0;
    IsjorB = 2'b0;
    DataRegWrite = 2'b10; // default
    DataRegWrite2 = 2'b0;

    LoadExtend = 3'b0;// don't extend Load if value is zero

    Immextend = 3'b0;

    // for R-type
    if (opcode == 7'b0110011) begin
      RegWrite = 1'b1;
      IsjorB = 2'b00; // update pc by 4 or if branch is taken branch

      // write to register the Alu result
      DataRegWrite2 = 2'b00;
      DataRegWrite = 2'b10;
      Immextend = 3'bx;



    end

    // for I-type

    else if (opcode == 7'b0010011) begin
      RegWrite = 1'b1;
      IsjorB = 2'b00; // update pc by 4 or if branch is taken branch

      // Sign-extend according to I-type and use immediate as second source
      Immextend = 3'b001;
      UseImm = 1'b1;


      // write to register the Alu result
      DataRegWrite2 = 2'b00;
      DataRegWrite = 2'b10; 


    end

    // for S-type,stores

    else if (opcode == 7'b0100011) begin
      MemWrite =1'b1;
      Immextend = 3'b010; // Sign-extend according to S-type
      UseImm = 1'b1; // use immediate as second source 

      IsjorB = 2'b00;// update pc by 4

    end

    // for Load

    else if (opcode == 7'b0000011) begin
      MemRead = 1'b1;
      RegWrite = 1'b1;

      // write to register the value that came from memory
      DataRegWrite2 = 2'b10;
      DataRegWrite = 2'b10;

      UseImm = 1'b1; // use immediate as second source
      Immextend = 3'b001; // Sign- extend according to I type

      IsjorB = 2'b00; // update PC by 4


      // depending upon what type of load it is , extract the right info
      if (funct3 == 3'b000) //LB
        LoadExtend = 3'b001;
      else if (funct3 == 3'b001) //LH
        LoadExtend = 3'b010;
      else if (funct3 == 3'b010) //LW
        LoadExtend = 3'b011;
      else if (funct3 == 3'b100) //LBU
        LoadExtend = 3'b100;
      else if (funct3 == 3'b101) //LHU
        LoadExtend = 3'b101;
      else
        LoadExtend = 3'b000;




    end

    // For U-type

    else if (opcode == 7'b0110111) begin // LUI

      IsjorB  = 2'b00; // Update PC by 4

      RegWrite = 1'b1;
      // write  imm << 12 to register
      DataRegWrite2 =  2'b01; 
      DataRegWrite = 2'b10;




    end

    else if (opcode == 7'b0010111) begin // AUIPC
      IsjorB = 2'b00; // update PC by 4

      RegWrite = 1'b1;

      // write  PC + (imm << 12)
      DataRegWrite = 2'b01;



      end

    else if (opcode == 7'b1101111) begin // JAL

      IsjorB = 2'b01; // update PC by PC + SIGN_EXTENDED
      RegWrite = 1'b1;
      DataRegWrite = 2'b00; // write PC + 4  to  dest register
      Immextend = 3'b011;  // Immediate extend according to J type




    end

    else if (opcode == 7'b1100111) begin // JALR
      IsjorB = 2'b10; // update PC by (Alu result & 0xfffffffe)
      RegWrite = 1'b1;
      DataRegWrite = 2'b00; // write PC + 4 to dest register

      Immextend = 3'b011; // Immediate extend according to J type
      UseImm = 1'b1; // Use immediate as second source

    end

    else if (opcode == 7'b1100011) begin // For branches
      isBranch = 1'b1; // Branch is high
      IsjorB = 2'b00;  // update PC if branch cond is met by some value

      UseImm = 1'b0; // use immediate value  to get bcond
      Immextend = 3'b100; // Immediate extend according to B type



    end










  end// always end

endmodule
