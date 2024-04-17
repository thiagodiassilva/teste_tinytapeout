
// Copyright 2023 Benedikt Muehlbachler JKU
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// 		http://www.apache.org/licenses/LICENSE−2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License
// 
// ################################################################
// Module tt_um_calculator_muehlbb.v
// - Performs 16-bit arithmetic and logic operations on data
// - Use module alu.v (16-bit arithmetic logic unit)
// ################################################################

`default_nettype none
`ifndef __CALCULATOR__
`define __CALCULATOR__

`include "alu.v"

module tt_um_fpu(
    input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,   // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
	// Inputs
	wire rst = ~rst_n; // reset
	wire [3:0] alu_sel = ui_in[3:0]; // alu operation selection
	/* verilator lint_off UNUSEDSIGNAL */
	wire [3:0] dummy1 = ui_in[7:4];
	wire dummy2 = ena;
	/* verilator lint_on UNUSEDSIGNAL */

	// Outputs
	reg [4:0] status_out; // status register for output
	assign uo_out[4:0] = status_out;
	
	reg [2:0] counter; // counter register
	assign uo_out[7:5] = counter;
	
	// IOs
	wire [7:0] data_in = uio_in; // input-port for operand a and b of calculation
	reg [7:0] data_out; // data-output-register (for the result of operation)
	assign uio_out = data_out;
	reg [7:0] inout_en; //IO-Port Enable Register (0x00 for input, 0xff for output)
	assign uio_oe = inout_en;

	// Create registers for data
	reg [3:0] sel_reg; // alu sel register for ALU
	reg [15:0] data_a; // operand a register for calculation
	reg [15:0] data_b; // operand b register for calculation
	wire [15:0] y; // y output result of operation
	wire [4:0] status_wire; // status output of ALU
	
	alu alu_1(sel_reg, data_a, data_b, y, status_wire); //16-bit alu for operations

    always @(posedge clk) begin
        /* verilator lint_off CASEINCOMPLETE */  
       	case(counter)
       		3'b001 : data_a[7:0] <= data_in; //save in low-byte of A
    		3'b010 : data_a[15:8] <= data_in; //save in high-byte of A
    		3'b011 : data_b[7:0] <= data_in; //save in low-byte of B
    		3'b100 : begin
    					data_b[15:8] <= data_in; //save in high-byte of B
    					sel_reg <= alu_sel; // select ALU operation
    					end
    		3'b101 : begin
    					data_out <= y[7:0]; //write low-byte of result on IO-Port
    					status_out <= status_wire; //write status-reg on output
    					end
    		3'b110 : data_out <= y[15:8]; //write high-byte of result on IO-Port
    	endcase
    	/* verilator lint_on CASEINCOMPLETE */
    end
    
    always @(negedge clk) begin
    	if (rst || counter == 3'b110) begin
    		// if reset or counter finished --> start from new
    		counter <= 3'b000; // counter=0
    		inout_en <= 8'h00; // set IO-Port as IN (to save a nd b in registers)
    	end	else
    		counter <= counter + 1;
    		
    	if (counter == 3'b100)
    		inout_en <= 8'hff; // set IO-Port as OUT (for result output)
   	end
endmodule

`endif
`default_nettype wire
// /*
//  * Copyright (c) 2024 Aravind-Prasad-Abhinav-Prakash
//  * SPDX-License-Identifier: Apache-2.0
//  */

// `define default_netname none

// module tt_um_fpu (
//     input  wire [7:0] ui_in,    // Dedicated inputs
//     output wire [7:0] uo_out,   // Dedicated outputs
//     input  wire [7:0] uio_in,   // IOs: Input path
//     output wire [7:0] uio_out,  // IOs: Output path
//     output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
//     input  wire       ena,      // will go high when the design is enabled
//     input  wire       clk,      // clock
//     input  wire       rst_n     // reset_n - low to reset
// );

  

   
// assign uio_oe = 8'b11111111;
  
//   wire [191:0]   inputRegister;
//   reg [31:0]   outputRegister;
  
// reg [31:0] I1; 
// reg [31:0] I2; 
// reg [31:0] I3;
// reg [31:0] I4;
// reg [31:0] I5;
// reg [31:0] I6;
// wire valid ; 
// wire [31:0] out ;
  
// FPA_6IP_old fpu_6ip( 
// .I1 (I1) ,
// .I2 (I2) ,
// .I3 (I3) ,
// .I4 (I4) ,
// .I5 (I5) ,
// .I6 (I6) ,
// .clk(clk)   ,
// .En (rst_n)   ,
// .valid(uio_out[4]),
// .out(out)
// );
    
//   write_data wd(
//     .clk             (clk),
//     .write_data_reset(rst_n),//active low reset
//     .data_in_wd         (ui_in), //8 bit input
//     .data_out_wd         (inputRegister), //144 bit output
//     .data_ready      (uio_out[0]),
//     .data_read       (uio_out[1])
// );


//   read_data rd(
//   .clk            (clk),
//   .read_data_reset(rst_n), //active low reset
//   //.data_in_rd     ({8'b11111111,inputRegister}), //152 bit input
//   .data_in_rd          (outputRegister), //152 bit input
//   .data_out_rd         (uo_out), //8 bit output
//   .input_changed  (uio_out[2]),
//   .enable_output  (uio_out[3])
// );



//   // Group inputs into input register
//   always @(posedge clk) begin
//         I1<= inputRegister[31:0];
//         I2<= inputRegister[63:32];
//         I3<= inputRegister[95:64];
//         I4<= inputRegister[127:96];
//         I5<= inputRegister[159:128];
//         I6<= inputRegister[191:160];
        
      
//   end

//   // Group outputs into output register
//   always @(posedge clk) begin
//     if (rst_n) begin
//       outputRegister <= 0;
//     end else begin
//       outputRegister <= out;
//     end
//   end

// endmodule




// module read_data #( parameter REG_WIDTH = 32)(
//   input wire clk,
//   input wire read_data_reset,  //active low reset
//   input wire [REG_WIDTH-1:0] data_in_rd,
//   output reg [7:0] data_out_rd,
//   output reg input_changed,  // will change when data_in is rewritten
//   output reg enable_output  // will be high when data is getting read from data_in and low when data is read
// );


//   reg [REG_WIDTH-1:0] internal_data;
//   reg [4:0] counter;

//   reg [REG_WIDTH-1:0] prev_data_in;

//   always @(posedge clk or posedge read_data_reset) begin
//     if (read_data_reset) begin
//       internal_data <= 0;
//       counter <= 0;
//       enable_output <= 0;
//       prev_data_in <= 0;
//       input_changed <= 0;
//     end else begin
//       if (enable_output) begin
//         if (counter < (REG_WIDTH/8)) begin
//           data_out_rd <= internal_data[8*counter +: 8];
//           counter <= counter + 1;
//         end else begin
//           enable_output <= 0;
//         end
//       end else begin
//         if (data_in_rd != prev_data_in) begin
//           internal_data <= data_in_rd;
//           enable_output <= 1;
//           counter <= 0;
//           input_changed <= ~input_changed;
//         end else begin
//           //input_changed <= 0;
//         end
//         prev_data_in <= data_in_rd;
//       end
//     end
//   end

// endmodule


// module write_data#(parameter REG_WIDTH = 192)(
//     input wire clk,
//     input wire write_data_reset, //active low reset
//     input wire [7:0] data_in_wd ,
//     output reg [REG_WIDTH-1:0] data_out_wd ,
//     output reg data_ready,  // Indicates when data_out is fully written
//     output reg data_read  // Indicates when data_out is fully read
// );
    
//     reg [4:0] write_counter = 0;  // Counts the number of writes
//     reg [REG_WIDTH-1:0] temp_data_out;  // Temporary register to store data_out

//     always @(posedge clk or posedge write_data_reset) begin
//         if (write_data_reset) begin
//             data_out_wd  <= 0;
//             write_counter <= 0;
//             data_ready <= 0;  // Clear the data_ready flag on reset
//             data_read <= 0;  // Clear the data_read flag on reset
//         end else if (write_counter < (REG_WIDTH/8)) begin
//             // Shift and write data
//             data_out_wd <= {data_out_wd [REG_WIDTH-9:0], data_in_wd };
//             write_counter <= write_counter + 1;
//             data_ready <= (write_counter == ((REG_WIDTH/8)-1));  // Set ready on the last write
//         end else if (data_read) begin
//             // Write new data only if previous data is successfully read
//             data_out_wd <= temp_data_out;
//             write_counter <= 0;
//             data_out_wd <=0;
//             data_ready <= 0;
//             data_read <= 0;
//         end
//     end

//     always @(posedge clk) begin
//         if (data_ready && ~data_read) begin
//             // Read data when data_ready is high
//             temp_data_out <= data_out_wd;
//             data_read <= 1;
//         end
//     end
// endmodule





// module FPA_6IP_old ( 
// input[31:0] I1 , 
// input[31:0] I2 , 
// input[31:0] I3,
// input[31:0] I4,
// input[31:0] I5,
// input[31:0] I6, 
// input clk,
// input En, 
// output valid, 
// output [31:0] out);

//    reg[31:0] input1,input2,input3,input4,input5,input6;
//    reg[31:0] op_wod,op_wod_reg;
//    reg valid_reg;
   
//    always @(*)
//    begin     
//      if(En) begin
//      input1 = I1;
//      input2 = I2;
//      input3 = I3;
//      input4 = I4;
//      input5 = I5;
//      input6 = I6;
//        //input1 <= 0;
//        //input2  <= 0;
      
//      end
//      else begin
//      input1 = 0;
//      input2 = 0;
//      input3 = 0;
//      input4 = 0;
//      input5 = 0;
//      input6 = 0;
// //      op_wod = 0;
// //       valid_reg=0;
//       end
//    end
   
    

//     wire S1,S2,S3,S4,S5,S6;
//     wire [7:0] E1,E2,E3,E4,E5,E6,Emax;
//     wire [7:0] EN1,EN2,EN3,EN4,EN5,EN6;
//     wire [31:0] M1,M2,M3,M4,M5,M6;
//     wire [31:0] N1,N2,N3,N4,N5,N6;
//     wire [31:0] complemented1,complemented2,complemented3,complemented4,complemented5,complemented6;
//     reg [7:0] E31;
//     reg[31:0] M31;
//     wire flag1,flag2,flag3,flag4,flag5,flag6;
    
//     splity_old SP1(input1,S1,E1,M1,flag1);
//     splity_old SP2(input2,S2,E2,M2,flag2);
//     splity_old SP3(input3,S3,E3,M3,flag3);
//     splity_old SP4(input4,S4,E4,M4,flag4);
//     splity_old SP5(input5,S5,E5,M5,flag5);
//     splity_old SP6(input6,S6,E6,M6,flag6);
//     find_greatest_old fa1(E1,E2,E3,E4,E5,E6,Emax);


//     wire  [7:0]E_Difference1,E_Difference2,E_Difference3,E_Difference4,E_Difference5,E_Difference6;
//     assign E_Difference1 = Emax - E1;
//     assign E_Difference2 = Emax - E2;
//     assign E_Difference3 = Emax - E3;
//     assign E_Difference4 = Emax - E4;
//     assign E_Difference5 = Emax - E5;
//     assign E_Difference6 = Emax - E6;


//     //assign N1 = {|E1,M1};   //Reduction OR handles zeroes
//     //assign N2 = {|E2,M2};   //and denormal numbers ... we just do the manual normalization.

//     shifty_old RS1( M1,E_Difference1,N1);  // makes sure exponents are of the same value.
//     shifty_old RS2( M2,E_Difference2,N2);  // makes sure exponents are of the same value.
//     shifty_old RS3( M3,E_Difference3,N3);  // makes sure exponents are of the same value.
//     shifty_old RS4( M4,E_Difference4,N4);  // makes sure exponents are of the same value.
//     shifty_old RS5( M5,E_Difference5,N5);  // makes sure exponents are of the same value.
//     shifty_old RS6( M6,E_Difference6,N6);  // makes sure exponents are of the same value.
    


//     check_sign_old Ch1(S1,N1,complemented1);
//     check_sign_old Ch2(S2,N2,complemented2);
//     check_sign_old Ch3(S3,N3,complemented3);
//     check_sign_old Ch4(S4,N4,complemented4);
//     check_sign_old Ch5(S5,N5,complemented5);
//     check_sign_old Ch6(S6,N6,complemented6);

//     wire [31:0] out1;
//     wire [31:0] sum_of_six;//,sum_of_six_2s;
// //    wire sign_value;
//     wire [4:0]msb_pos;
//     wire right_left;

// //    signed_adder_32bit aa (complemented1,complemented2,complemented3,complemented4,complemented5,complemented6, out1,sign_value);
//    assign  out1 = complemented1+complemented2+complemented3+complemented4+complemented5+complemented6;
// //    final_complement fc (sign_value, out1,sum_of_six);
//     //assign sum_of_six_2s = (~sum_of_six)+1 ;
//      assign sum_of_six = (out1[31] == 1) ? (~out1 + 1) : out1;
//     check_msb_old dt (sum_of_six,msb_pos);
//     assign right_left = (msb_pos > 23)?1:0;

//      always @(*) // normalizing and adjusting , step 6 .
//     begin
//     if(!En || flag1 || flag2 ||flag3 || flag4 || flag5 || flag6)begin
//      M31 =0;
//       E31 =0;
//        op_wod_reg =0;
//     end
//     else begin
//         if(right_left == 1)
//         begin
//             M31 = sum_of_six >> (msb_pos-23);
//             E31 =  (Emax + (msb_pos-23)) ;
//             op_wod_reg = {out1[31],E31,M31[22:0]};
//         end
//         else 
//         begin
//             M31 = sum_of_six << (23-msb_pos);
//             E31 =  (Emax - (23-msb_pos)) ;
//             op_wod_reg = {out1[31],E31,M31[22:0]};
//         end
//        end 
//    end

//     always @ (posedge clk)   //  always @ (E3 or M3)
//     begin
// //        if (!En) begin 
// //       op_wod<=0;
        
// //        end
// //        else
// //        begin
// ////        if (input1==0 && input2==0) op_wod=0;
// //////            op_wod <= {sign_value,E31,M31[22:0]};//Handles normal + NaN
// ////        else     
//          op_wod<= op_wod_reg;
// //          valid_reg=1;
// //           change<=1;
             
// //            end
//     end
//     //
//  always@(op_wod,En )
//    begin
//    if(En)
//    begin
//            if (|op_wod)
//            valid_reg=1;
//              else valid_reg=0;
//    end
//    else valid_reg=0;
//    end
  
// assign out=op_wod;
// assign valid=valid_reg;



// endmodule


   

// module splity_old(  input [31:0]A, output reg sign, output reg [7:0]exp, output reg [31:0]man,output reg flag); // partitioning the 32 bits 
// //    assign sign = A[31];
// //    assign exp = A [30:23];
// //    assign man =  (A [30:0]==31'b0)?32'b0:{9'b00000001,A[22:0]};
// //reg [31:0]man;
// //reg flag;
// //reg [7:0]exp;
//     always@(*)
//     begin
//     if ( A[30:0]==0 )
//       begin
//         sign = 0;
//         exp = 8'b0;
//         man = {9'b00000000,23'b0}; //exact 0
//         flag=0;
//       end
//       else if(A[30:23]==8'hff && A[22:0]==23'b0 )
//       begin
//          sign =0;
//         exp = 8'b0;
//          man = {9'b00000000,23'b0};  //infinity
//         flag=1;
//       end
//       else if ( A[30:23]==8'b0 && A[22:0]!=23'b0 )
//       begin
//       sign = 0;
//        exp = 8'b0;
//         man = {9'b00000000,23'b0};  //denormalized
//         flag=1;
//       end
//        else if ( A[30:23]==8'b1 && A[22:0]!=23'b0 )
//       begin
//        sign =0;
//        exp = 8'b0;
//         man = {9'b00000000,23'b0};  //NAN
//         flag=1;
//       end
//       else
//       begin
//       sign = A[31];
//       exp = A[30:23];
//       man={9'b00000001,A[22:0]};
//       flag=0;
//       end
      
//     end
    
    
// endmodule




// module find_greatest_old (
//   input [7:0] input_1,
//   input [7:0] input_2,
//   input [7:0] input_3,
//   input [7:0] input_4,
//   input [7:0] input_5,
//   input [7:0] input_6,
//   output reg [7:0] greatest_output
// );
 
// reg [7:0] A;
// reg [7:0] B;
// reg [7:0] C;
// reg [7:0] D;
// reg [7:0] E;
// reg [7:0] F;
//   always @* begin
//    A = 8'b00000000;
    
// //    if (input_1 > greatest_output) greatest_output = input_1;
// //    if (input_2 > greatest_output) greatest_output = input_2;
// //    if (input_3 > greatest_output) greatest_output = input_3;
// //    if (input_4 > greatest_output) greatest_output = input_4;
// //    if (input_5 > greatest_output) greatest_output = input_5;
// //    if (input_6 > greatest_output) greatest_output = input_6;
    
//     B = (A > input_1) ? A : input_1;
//     C = (B > input_2) ? B : input_2;
//     D = (C > input_3) ? C : input_3;
//     E = (D > input_4) ? D : input_4;
//     F = (E > input_5) ? E : input_5;
//     greatest_output = (F > input_6) ? F : input_6;
     
    
    
//   end

// endmodule




// module shifty_old (input [31:0]B, input [7:0]shift,  output reg [31:0]OB);
// //reg [7:0]O;
// always@*
// begin
//     if(shift!=0)
//     begin
//     OB = B>>shift;
// //    O = A+shift;
    
//     end
//     else
//     begin
// //    O=A;
//     OB=B;
//     end
// end
// endmodule



// module check_sign_old (
//   input wire S,
//   input wire [31:0] padded_N,
//   output reg [31:0] complemented
// );

//   always @* begin
//     if (S == 1'b1) begin
//       // If S1 is 1, complemented1 is the two's complement of padded_N1
//       complemented = ~padded_N + 1;
//     end
//     else begin
//       // If S1 is 0, complemented1 is equal to padded_N1
//       complemented = padded_N;
//     end
//   end

// endmodule

// module check_msb_old(
//   input [31:0] data,
//   output reg [4:0] msb_position
// );

//   always @* begin
//     msb_position = 5'b0;

//     // Check the upper 16 bits
//     if (data[31]) msb_position = 5'd31;
//     else if (data[30]) msb_position = 5'd30;
//     else if (data[29]) msb_position = 5'd29;
//     else if (data[28]) msb_position = 5'd28;
//     else if (data[27]) msb_position = 5'd27;
//     else if (data[26]) msb_position = 5'd26;
//     else if (data[25]) msb_position = 5'd25;
//     else if (data[24]) msb_position = 5'd24;
//     else if (data[23]) msb_position = 5'd23;
//     else if (data[22]) msb_position = 5'd22;
//     else if (data[21]) msb_position = 5'd21;
//     else if (data[20]) msb_position = 5'd20;
//     else if (data[19]) msb_position = 5'd19;
//     else if (data[18]) msb_position = 5'd18;
//     else if (data[17]) msb_position = 5'd17;
//     else if (data[16]) msb_position = 5'd16;
//     else if (data[15]) msb_position = 5'd15;
//     else if (data[14]) msb_position = 5'd14;
//     else if (data[13]) msb_position = 5'd13;
//     else if (data[12]) msb_position = 5'd12;
//     else if (data[11]) msb_position = 5'd11;
//     else if (data[10]) msb_position = 5'd10;
//     else if (data[9]) msb_position = 5'd9;
//     else if (data[8]) msb_position = 5'd8;
//     else if (data[7]) msb_position = 5'd7;
//     else if (data[6]) msb_position = 5'd6;
//     else if (data[5]) msb_position = 5'd5;
//     else if (data[4]) msb_position = 5'd4;
//     else if (data[3]) msb_position = 5'd3;
//     else if (data[2]) msb_position = 5'd2; 
//     else if (data[1]) msb_position = 5'd1; 
//     else if (data[0]) msb_position = 5'd0;
//     else msb_position = 5'd0;
        

//   end

// endmodule
