
module ALU(input_one,input_two,opcode,funct3,funct7,Alu_result,bcond);
  input [31:0] input_one,input_two;
  input [6:0] opcode,funct7;
  input [2:0] funct3;
  output reg [31:0] Alu_result;
  output reg bcond;
  reg signed [31:0] Alu_in1,Alu_in2;
  reg  [31:0] input_o,input_tw;
  reg [6:0] opcod,funct72;
  reg [2:0] funct32;
  always @(opcode)
  opcod = opcode;
  always @(funct3)
  funct32 = funct3;
  always @(funct7)
  funct72 = funct7;
  
   always @(input_one) begin
  Alu_in1 = input_one;
  input_o = input_one;
   end
   
   always @(input_two) begin
  Alu_in2 = input_two;
  input_tw = input_two;
    end
 
  always @(*) begin

    if (opcod == 7'b0110011) begin // R-type 
      bcond = 0;
      case(funct32)
        3'b000: begin // add and subtract
          if (funct72 == 0)
             Alu_result = Alu_in1 + Alu_in2;
          else
            Alu_result = Alu_in1 - Alu_in2;

        end
        3'b001: // left shift logical
          Alu_result = (input_o << input_tw[4:0]);
        3'b010: // Set less than
          Alu_result = (Alu_in1 < Alu_in2);
        3'b011:// Set less than unsigned
          Alu_result = (input_o < input_tw);
        3'b100: // xor
          Alu_result = (input_o ^ input_tw);
        3'b101: begin // SRL and SRA
          if (funct72 == 0) // SRL
            Alu_result = (input_o >> input_tw[4:0]);
          else
            Alu_result = (Alu_in1 >> input_tw[4:0]);
        end
        3'b110: // OR
          Alu_result = (input_o | input_tw);
        3'b111://AND
          Alu_result = (input_o & input_tw);
       endcase
     end // if -end
    else if (opcod == 7'b0010011) begin // immediates but doesnt include load
      bcond = 0;
      case(funct32)
        3'b000: //ADDI
          Alu_result  = (Alu_in1 + Alu_in2);
        3'b010: // SLTI
          Alu_result = (Alu_in1 < Alu_in2);
        3'b011: // SLTIU
          Alu_result = (input_o < input_tw);
        3'b100: // xori
          Alu_result = (input_o ^ input_tw);
        3'b110: // ORI
          Alu_result = (input_o | input_tw);
        3'b111://ANDI
          Alu_result = (input_o & input_tw);
        3'b001: //SLLI, take care of the sign extending part ** tricky
          Alu_result = (input_o << input_tw[4:0]); // shamt
        3'b101: //SRLI , this are also tricky, funct7 and ahmt look at the manual when giving value
          begin
            if (funct72 ==0)
              Alu_result = (input_o >> input_tw[4:0]); // shamt
            else
              Alu_result = (Alu_in1 >> input_tw[4:0]);//shamt
          end

      endcase
       end // end else if

   
    else if(opcod == 7'b0000011 || opcod == 7'b0100011 || opcod == 7'b1100111 || opcod == 7'b1101111) // load or store or JALR or JAL
        begin
        bcond = 0;
        Alu_result = Alu_in1 + Alu_in2;
        end // else if end
    else if (opcod == 7'b1100011) begin // branch
      Alu_result = 32'bx;
      case(funct32)
        3'b000: begin // BEQ
          if (Alu_in1 == Alu_in2)
            bcond = 1;
          else
            bcond = 0;
        end
        3'b001: begin // BNE
          if (Alu_in1 != Alu_in2)
            bcond = 1;
          else
            bcond = 0;
        end
        3'b100: begin // BLT
          if (Alu_in1 < Alu_in2)
            bcond = 1;
          else
            bcond = 0;
        end
        3'b101: begin // BGE
          if (Alu_in1 >= Alu_in2)
            bcond = 1;
          else
            bcond = 0;
        end
        3'b110: begin // BLTU
          if (input_o < input_tw)
            bcond = 1;
          else
            bcond = 0;
        end
        3'b111: begin // BGEU
          if (input_o >= input_tw)
            bcond = 1;
          else
            bcond = 0;
        end
        default: bcond = 0;

      endcase


    end // else if end



        end // always - end

endmodule


