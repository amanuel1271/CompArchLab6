module forwardAndStall(IF_ID ,ID_EX,EX_MEM ,stall_flag ,AluSrc_A ,AluSrc_B);
  input [31:0] IF_ID ,ID_EX,EX_MEM;
  output reg [1:0] AluSrc_A ,AluSrc_B;
  output reg stall_flag;
  reg [31:0] IForID ,IDorEX,EXorMEM;
  reg WriteEnable2,WriteEnable3;
  reg ReadEnable;
  reg [6:0]opcode1,opcode2,opcode3;
  reg [4:0] destReg2;
  reg [4:0] destReg3;
  reg [4:0] RsrcA;
  reg [4:0] RsrcB;  
  always @(IF_ID)begin
	IForID = IF_ID;
  	opcode1 = IForID[6:0];
  //register reading instructions
  //////////////////////////////
  if(opcode1 == 7'b0000011 || opcode1 == 7'b0110011 || opcode1 == 7'b1100111 || opcode1 == 7'b0010011 || opcode1 == 7'b1100011 || opcode1 == 7'b0100011 ) // load or ALU or or JALR or ALUI or branch or store
        begin
        ReadEnable = 1;
          if(opcode1==7'b0000011 || opcode1==7'b0010011 || opcode1==7'b1100111)begin //load or alui or jalr
            RsrcA = IForID[19:15]; // read reg 1
            RsrcB = 5'bx;
          end

          else
            begin
            RsrcA = IForID[19:15]; // read reg 1
            RsrcB = IForID[24:20]; // read reg 2
          end  
        end
  else begin
    ReadEnable =0;
    RsrcA = 5'bx; // read reg 1
    RsrcB = 5'bx;
  end
  end
  //////////////////////////////////////////////////////////
  always @(ID_EX)
    begin
	IDorEX = ID_EX;
  	opcode2 = IDorEX[6:0];
    WriteEnable2 = (opcode2 == 7'b0000011 || opcode2 == 7'b0110011 || opcode2 == 7'b1100111 || opcode2 == 7'b0010011 || opcode2 == 7'b1101111);// load,ALU,JALR,ALUI,JAL
  	if(WriteEnable2)
      destReg2 = IDorEX[11:7];
	else
	destReg2 = 5'bx;
    end 
  always @(EX_MEM)
    begin
	EXorMEM = EX_MEM;
  	opcode3 = EXorMEM[6:0];
    WriteEnable3 = (opcode3 == 7'b0000011 || opcode3 == 7'b0110011 || opcode3 == 7'b1100111 || opcode3 == 7'b0010011 || opcode3 == 7'b1101111 );// load,ALU,JALR,ALUI,JAL
  	if(WriteEnable3)
      destReg3 = EXorMEM[11:7];
	else
	destReg3 = 5'bx;
    end
  always @(*)
    begin
    if(opcode2 == 7'b0000011 && (RsrcA===destReg2 || RsrcB === destReg2) && destReg2!=0 && ReadEnable)begin
      stall_flag = 1;
      AluSrc_A = 2'b00;
      AluSrc_B = 2'b00;
    end
  	else
      begin
        stall_flag = 0;
        if(ReadEnable && (WriteEnable2 || WriteEnable3))begin
          if(RsrcA === destReg3 && WriteEnable3 && destReg3!== 5'bx)
          	AluSrc_A =2'b01;
	  else
		AluSrc_A = 2'b00;

          if(RsrcA === destReg2 && WriteEnable2)
            AluSrc_A =2'b10;

          if( (RsrcA!==destReg3) &&  (RsrcA!==destReg2))
            AluSrc_A = 2'b00;
          
          ///logic for ALU b
          if(RsrcB === destReg3 && destReg3!==5'bx && opcode1 !== 7'b0100011) // store, is messing  everything
          	AluSrc_B =2'b01;
	  else
		AluSrc_B = 2'b00;
          if(RsrcB === destReg2 && WriteEnable2)
            AluSrc_B = 2'b10;
	  if (RsrcB === destReg2 && WriteEnable2 && opcode1 === 7'b0100011)// store, tebeda
		AluSrc_B = 2'b00;
	  
          if(RsrcB!==destReg3 && RsrcB!==destReg2)
            AluSrc_B = 2'b00;
          end
        // take the normal path
         else
           begin
           AluSrc_A = 2'b00;
           AluSrc_B = 2'b00;
           end
         end
      end
endmodule