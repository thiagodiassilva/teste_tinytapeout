/*
 * Copyright (c) 2024 Aravind-Prasad-Abhinav-Prakash
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_fpu(
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;

endmodule


`timescale 1ns / 1ps

module tt_2ip_mult(

    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
///verilator lint_off UNUSEDSIGNAL/
    input  wire [7:0] uio_in,   // IOs: Input path - only some bits used
//verilator lint_on UNUSEDSIGNAL/
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
//verilator lint_off UNUSEDSIGNAL/
    input  wire       ena,
//verilator lint_on UNUSEDSIGNAL/
    input  wire       clk,
    input  wire       rst_n
);
assign uio_oe = 8'b11111111;
  
  wire [63:0]   inputRegister;
  reg [63:0]   outputRegister;
  wire data_ready , data_read, input_changed,enable_output;
//reg [31:0] I1; 
//reg [31:0] I2; 

//wire valid ; 
//wire [31:0] out ;
  
//My_FPMult fpu_2ip_m( 
//.I1 (I1) ,
//.I2 (I2) ,
//.clk(clk)   ,
//.rst(rst_n),
//.out(out)
//);
    
  write_data wd(
    .clk             (clk),
    .write_data_reset(rst_n),//active low reset
    .data_in_wd         (ui_in), //8 bit input
    .data_out_wd         (inputRegister), //144 bit output
    .data_ready      (data_ready),
    .data_read       (data_read)
);


  read_data rd(
  .clk            (clk),
  .read_data_reset(rst_n), //active low reset
  .data_in_rd     (outputRegister), //152 bit input
  //.data_in_rd          (outputRegister), //152 bit input
  .data_out_rd         (uo_out), //8 bit output
  .input_changed  (uio_out[2]),
  .enable_output  (uio_out[3])
);

assign uio_out = {data_ready , data_read, input_changed,enable_output};

  // Group inputs into input register
//  always @(posedge clk) begin
//        I1<= inputRegister[31:0];
//        I2<= inputRegister[63:32];
        
        
      
//  end

  // Group outputs into output register
  always @(posedge clk) begin
    if (rst_n) begin
      outputRegister <= 0;
    end else begin
        if (data_read && data_ready)begin
            outputRegister <= inputRegister;
            end
    end
  end

endmodule
