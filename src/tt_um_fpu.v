/*
 * Copyright (c) 2024 Aravind-Prasad-Abhinav-Prakash
 * SPDX-License-Identifier: Apache-2.0
 */



module tt_um_fpu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);


assign uio_oe = 8'b11111111;
  
  wire [127:0]   inputRegister;
  reg [31:0]   outputRegister = 0;
  wire data_ready , data_read, input_changed,enable_output;
  wire add_valid;

reg [31:0] I1 = 0 ;
reg [31:0] I2 = 0 ;
reg [31:0] I3 = 0 ;
reg [31:0] I4 = 0 ;



wire [31:0] out_fpu;

  FPU_TOP  fpu_t(
.I1 (I1), 
.I2 (I2), 
.I3 (I3), 
.I4 (I4),
.clk(clk), 
.rst(rst_n), 
.out_final(out_fpu),
.add_valid(add_valid)
);
    
  write_data wd(
    .clk             (clk),
    .write_data_reset(~rst_n),//active low reset
    .data_in_wd       (ui_in), //8 bit input
    .data_out_wd      (inputRegister), //128 bit output
    .data_ready      (data_ready),
    .data_read       (data_read)
);


  read_data rd(
  .clk            (clk),
  .read_data_reset(~rst_n), //active low reset
  .data_in_rd     (outputRegister), //32 bit input
  .data_out_rd    (uo_out), //8 bit output
  .input_changed  (input_changed),
  .enable_output  (enable_output)
);

assign uio_out = {4'b0000,data_ready,data_read, input_changed,enable_output};

  // Group inputs into input register
  always @(posedge clk) begin
      if (~rst_n) begin
        I1<= 0;
        I2<= 0;
        I3<= 0;
        I4<= 0;
      end else begin
        if (data_read && data_ready)begin
            I1<= inputRegister[31:0];
            I2<= inputRegister[63:32];
            I3<= inputRegister[95:64];
            I4<= inputRegister[127:96];
            end
//        else begin
//        I1<= 0;
//        I2<= 0;
//        I3<= 0;
//        I4<= 0;
//        end
                   
     end
  end

  // Group outputs into output register
  always @(posedge clk) begin
    if (~rst_n) begin
      outputRegister <= 0;
    end else begin
        if (add_valid)begin
            outputRegister <= out_fpu;
            end
//        else begin
//            outputRegister <= 0;
//        end       
    end
  end

endmodule

module write_data#(parameter REG_WIDTH = 128)(
    input wire clk,
    input wire write_data_reset, //active low reset
    input wire [7:0] data_in_wd,
    output reg [REG_WIDTH-1:0] data_out_wd,
    output reg data_ready,  // Indicates when data_out is fully written
    output reg data_read  // Indicates when data_out is fully read
);
    
    reg [4:0] write_counter = 0;  // Counts the number of writes
    reg [REG_WIDTH-1:0] temp_data_out =0;  // Temporary register to store data_out

    always @(posedge clk or posedge write_data_reset) begin
        if (write_data_reset) begin
            data_out_wd <= 0;
            write_counter <= 0;
            data_ready <= 0;  // Clear the data_ready flag on reset
            data_read <= 0;  // Clear the data_read flag on reset
        end else begin
            if (write_counter < (REG_WIDTH/8)) begin
                // Shift and write data
                data_out_wd <= {data_out_wd[REG_WIDTH-9:0], data_in_wd};
                write_counter <= write_counter + 1;
                data_ready <= (write_counter == ((REG_WIDTH/8)-1));  // Set ready on the last write
            end else if (data_read) begin
                // Write new data only if previous data is successfully read
                data_out_wd <= temp_data_out;
                write_counter <= 0;
                data_out_wd <= 0;
                data_ready <= 0;
                data_read <= 0;
            end
            if (data_ready && ~data_read) begin
                // Read data when data_ready is high
                temp_data_out <= data_out_wd;
                data_read <= 1;
            end
        end
    end
endmodule


module read_data #( parameter REG_WIDTH = 32)(
  input wire clk,
  input wire read_data_reset,  //active low reset
  input wire [REG_WIDTH-1:0] data_in_rd,
  output reg [7:0] data_out_rd,
  output reg input_changed,  // will change when data_in is rewritten
  output reg enable_output  // will be high when data is getting read from data_in and low when data is read
);

  reg [REG_WIDTH-1:0] internal_data = 0;
  reg [4:0] counter = 0 ;

  reg [REG_WIDTH-1:0] prev_data_in =0;

  always @(posedge clk or posedge read_data_reset) begin
    if (read_data_reset) begin
      internal_data <= 0;
      counter <= 0;
      enable_output <= 0;
      prev_data_in <= 0;
      input_changed <= 0;
      data_out_rd <=0;
    end else begin
          if (enable_output) begin
                if (counter < (REG_WIDTH/8)) begin
                 // data_out_rd <= internal_data[8*counter +: 8];
                 data_out_rd <= internal_data[((REG_WIDTH-1) - 8*counter) -: 8];
                 counter <= counter + 1;
                end else begin
                  enable_output <= 0;
                end
          end else begin
                if (data_in_rd != prev_data_in) begin
                  internal_data <= data_in_rd;
                  enable_output <= 1;
                  counter <= 0;
                  input_changed <= ~input_changed;
                end else begin
                  //input_changed <= 0;
                end
            prev_data_in <= data_in_rd;
          end
    end
  end

endmodule


module FPU_TOP(
input[31:0] I1 , 
input[31:0] I2 , 
input[31:0] I3 , 
input[31:0] I4 ,
input clk, 
input rst, 
output  [31:0] out_final,
output add_valid
);


wire [31:0]I12,I34;


My_FPMult MUL12(.I1(I1),.I2(I2),.clk(clk),.rst (rst),.out(I12));  
My_FPMult MUL23(.I1(I3),.I2(I4),.clk(clk),.rst (rst),.out(I34));  
FPA_2IP_old ADD56(.I1(I12),.I2(I34),.clk(clk),.En(rst),.valid(add_valid),.out(out_final));       

endmodule

module My_FPMult ( input[31:0] I1 , input[31:0] I2 , input clk, input rst, output  [31:0] out );          //   Multiplier with latency of 7 clock cycles

   reg[31:0] input1, input2;
   reg[31:0] op_wod;

  always @(posedge clk)
   begin
     if(!rst) begin
       input1 <= 0;
       input2 <= 0;
     end
     else begin
       input1 <= I1;
       input2 <= I2;  
   end
  end

    wire S1,S2,S3;          //Signs
    wire [7:0] E1,E2;       //Exponents
    wire [22:0] M1,M2;      //Mantissa

    reg [22:0]M3;       //Final Mantissa
    reg [7:0]E3;        //Final Exponent

    //SPLIT phase
    Split SP1(input1,S1,E1,M1);
    Split SP2(input2,S2,E2,M2);

    //MULTIPLICATION
    wire [31:0] N1,N2;   //TEMPORARY VAR
    wire [63:0] N3;
    wire [8:0] temp_E3;

    assign N1 = {{8'b0},|E1,M1};   //Reduction xor for handle zeroes
    assign N2 = {{8'b0},|E2,M2};   //and denormal numbers
   

    assign temp_E3=E1+E2-127;
    assign S3=S1^S2;
    
    assign N3=N1*N2;

    //NORMALISING
    always@(*)  
    begin
        if(temp_E3>=255) begin 
            E3=8'b11111111;
            if (|M1 == 1'b0 || |M2 == 1'b0) M3=23'b0;
            else M3 = N3[46:24];
        end
        else if(N3[47]==1)
        begin
            M3=N3[46:24];
            E3=temp_E3+1;
        end
        else
        begin
            M3=N3[45:23]; //rounding off phase
            E3=temp_E3;
        end
        
    end

    //Checking various cases
    always@(*)
     
    begin
        if(!rst) op_wod = 0;
        else if(&E1 == 1'b1 && |M1 == 1'b0)          //INFINITY
        begin
            op_wod ={1'b0,8'b11111111,23'b0};
            
            end
        else if(&E2 == 1'b1 && |M2 == 1'b0)     //INFINITY
        begin
            op_wod ={1'b0,8'b11111111,23'b0};
             
            end

        else if(|E1 == 1'b0 && |M1 == 1'b0)     //ZERO
            begin
            op_wod ={32'b0};
             
            end
        else if(|E2 == 1'b0 && |M2 == 1'b0)     //ZERO
           begin
            op_wod ={32'b0};
             
            end
//        else if(op_wod_1[30:0]<30'b0110101100001100011011110111101)   //Threshold 0.000001
//           begin
//            op_wod ={32'b0};
            
//            end
        else                                   //NORMAL CASE
           begin
            op_wod = {S3,E3,M3};
            
            end
    end
      
   
   
    
assign out=op_wod;
endmodule


module Split(input [31:0]A, output sign, output [7:0]exp, output [22:0]man); // partitioning the 32 bits 
    assign sign = A[31];
    assign exp = A [30:23];
    assign man = A[22:0];
endmodule

module FPA_2IP_old( 
input[31:0] I1 , 
input[31:0] I2 ,  
input clk,
input En, 
output valid, 
output [31:0] out);

   reg[31:0] input1,input2;
   reg[31:0] op_wod,op_wod_reg;
   reg valid_reg;
   wire flag1,flag2;

   always @(*)
   begin     
     if(En) begin
     input1 = I1;
     input2 = I2;  
      
     end
     else begin
     input1 = 0;
     input2 = 0;
     
      end
   end
   
    

    wire S1,S2;
    wire [7:0] E1,E2,Emax;
//    wire [7:0] EN1,EN2;
    wire [31:0] M1,M2;
    wire [31:0] N1,N2;
    wire [31:0] complemented1,complemented2;
    reg [7:0] E31;
    reg[31:0] M31;
    
    splity_old SP1( input1,S1,E1,M1,flag1);
    splity_old SP2( input2,S2,E2,M2,flag2);
    
     
    assign Emax=(En==1)?((E1>=E2)? E1:E2):0;

    wire  [7:0]E_Difference1,E_Difference2;
    assign E_Difference1 = Emax - E1;
    assign E_Difference2 = Emax - E2;

     shifty_old RS1(  M1,E_Difference1,N1);  // makes sure exponents are of the same value.
     shifty_old RS2( M2,E_Difference2,N2);


    check_sign_old Ch1(S1,N1,complemented1);
    check_sign_old Ch2(S2,N2,complemented2);
   

    wire [31:0] out1;
    wire [31:0] sum_of_two;

    wire [4:0]msb_pos;
    wire right_left;
    
    assign out1=complemented1+complemented2;

    assign sum_of_two = (out1[31] == 1) ? (~out1 + 1) : out1;

    check_msb_old dt (sum_of_two,msb_pos);
    assign right_left = (msb_pos > 23)?1:0;

    always @(*) // normalizing and adjusting , step 6 .
    begin
    if(!En || flag1 || flag2 )begin
     M31 =0;
      E31 =0;
       op_wod_reg =0;
    end
    else begin
        if(right_left == 1 )
        begin
            M31 = sum_of_two >> (msb_pos-23);
            E31 =  (Emax + (msb_pos-23))  ;

            op_wod_reg = {out1[31],E31,M31[22:0]};
        end
        else 
        begin
            M31 = sum_of_two << (23-msb_pos);
            E31 =  (Emax - (23-msb_pos))  ;

            op_wod_reg = {out1[31],E31,M31[22:0]};
        end
       end 
   end

    always @ (posedge clk)   //  always @ (E3 or M3)
    begin
  
         op_wod<= op_wod_reg;

    end

    always@(op_wod,En )
   begin
   if(En)
   begin
           if (|op_wod)
           valid_reg=1;
             else valid_reg=0;
   end
   else valid_reg=0;
   end

  
assign out=op_wod;
assign valid=valid_reg;



endmodule

module splity_old(  input [31:0]A, output reg sign, output reg [7:0]exp, output reg [31:0]man,output reg flag); // partitioning the 32 bits 

    always@(*)
    begin
    if ( A[30:0]==0 )
      begin
        sign = 0;
        exp = 8'b0;
        man = {9'b00000000,23'b0}; //exact 0
        flag=0;
      end
      else if(A[30:23]==8'hff && A[22:0]==23'b0 )
      begin
         sign =0;
        exp = 8'b0;
         man = {9'b00000000,23'b0};  //infinity
        flag=1;
      end
      else if ( A[30:23]==8'b0 && A[22:0]!=23'b0 )
      begin
      sign = 0;
       exp = 8'b0;
        man = {9'b00000000,23'b0};  //denormalized
        flag=1;
      end
       else if ( A[30:23]==8'b1 && A[22:0]!=23'b0 )
      begin
       sign =0;
       exp = 8'b0;
        man = {9'b00000000,23'b0};  //NAN
        flag=1;
      end
      else
      begin
      sign = A[31];
      exp = A[30:23];
      man={9'b00000001,A[22:0]};
      flag=0;
      end
      
    end
    
    
endmodule

module shifty_old (input [31:0]B, input [7:0]shift,  output reg [31:0]OB);
//reg [7:0]O;
always@*
begin
    if(shift!=0)
    begin
    OB = B>>shift;
//    O = A+shift;
    
    end
    else
    begin
//    O=A;
    OB=B;
    end
end
endmodule


module check_sign_old (
  input wire S,
  input wire [31:0] padded_N,
  output reg [31:0] complemented
);

  always @* begin
    if (S == 1'b1) begin
      // If S1 is 1, complemented1 is the two's complement of padded_N1
      complemented = ~padded_N + 1;
    end
    else begin
      // If S1 is 0, complemented1 is equal to padded_N1
      complemented = padded_N;
    end
  end

endmodule


module check_msb_old(
  input [31:0] data,
  output reg [4:0] msb_position
);

  always @* begin
    msb_position = 5'b0;

    // Check the upper 16 bits
    if (data[31]) msb_position = 5'd31;
    else if (data[30]) msb_position = 5'd30;
    else if (data[29]) msb_position = 5'd29;
    else if (data[28]) msb_position = 5'd28;
    else if (data[27]) msb_position = 5'd27;
    else if (data[26]) msb_position = 5'd26;
    else if (data[25]) msb_position = 5'd25;
    else if (data[24]) msb_position = 5'd24;
    else if (data[23]) msb_position = 5'd23;
    else if (data[22]) msb_position = 5'd22;
    else if (data[21]) msb_position = 5'd21;
    else if (data[20]) msb_position = 5'd20;
    else if (data[19]) msb_position = 5'd19;
    else if (data[18]) msb_position = 5'd18;
    else if (data[17]) msb_position = 5'd17;
    else if (data[16]) msb_position = 5'd16;
    else if (data[15]) msb_position = 5'd15;
    else if (data[14]) msb_position = 5'd14;
    else if (data[13]) msb_position = 5'd13;
    else if (data[12]) msb_position = 5'd12;
    else if (data[11]) msb_position = 5'd11;
    else if (data[10]) msb_position = 5'd10;
    else if (data[9]) msb_position = 5'd9;
    else if (data[8]) msb_position = 5'd8;
    else if (data[7]) msb_position = 5'd7;
    else if (data[6]) msb_position = 5'd6;
    else if (data[5]) msb_position = 5'd5;
    else if (data[4]) msb_position = 5'd4;
    else if (data[3]) msb_position = 5'd3;
    else if (data[2]) msb_position = 5'd2; 
    else if (data[1]) msb_position = 5'd1; 
    else if (data[0]) msb_position = 5'd0;
    else msb_position = 5'd0;
        

  end

endmodule
   



