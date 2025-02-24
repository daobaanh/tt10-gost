//======================================================================
//
// gost_28147_89.v
// --------------------
// The GOST 28147 89 encipher/decipher round.
//
//
// Author: Ba-Anh Dao
// Copyright (c) 2023, Vien NCUD KH&CN - Hoc Vien KTMM
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

`define Sbox(x) {S8(x[31:28]),S7(x[27:24]),S6(x[23:20]),S5(x[19:16]),S4(x[15:12]),S3(x[11:8]),S2(x[7:4]),S1(x[3:0])}


module gost_28147_89 (clk, rst, mode, load, done, key, pdata, cdata);
  input  clk;    // Input clock signal for synchronous design
  input  rst;    // Syncronous Reset input
  input  mode;   // 0 - encrypt, 1 - decrypt
  input  load;   // load plain text and start cipher cycles
  output done;   // cipher text ready for output read
  //input  kload;  // load cipher key
  input [255:0] key;   // cipher key input
  input  [63:0] pdata; //  plain text input
  output [63:0] cdata; // cipher text output

//`include "gost-sbox.vh"
//`include "sbox.vh"

function [3:0] S1( input [3:0] x );
  begin
    case(x)
      4'd00: S1 = 4'h4;
      4'd01: S1 = 4'hA;
      4'd02: S1 = 4'h9;
      4'd03: S1 = 4'h2;
      4'd04: S1 = 4'hD;
      4'd05: S1 = 4'h8;
      4'd06: S1 = 4'h0;
      4'd07: S1 = 4'hE;
      4'd08: S1 = 4'h6;
      4'd09: S1 = 4'hB;
      4'd10: S1 = 4'h1;
      4'd11: S1 = 4'hC;
      4'd12: S1 = 4'h7;
      4'd13: S1 = 4'hF;
      4'd14: S1 = 4'h5;
      4'd15: S1 = 4'h3;
      default: S1 = 4'hX;
    endcase
  end
endfunction


function [3:0] S2( input [3:0] x );
  begin
    case(x)
      4'd00: S2 = 4'hE;
      4'd01: S2 = 4'hB;
      4'd02: S2 = 4'h4;
      4'd03: S2 = 4'hC;
      4'd04: S2 = 4'h6;
      4'd05: S2 = 4'hD;
      4'd06: S2 = 4'hF;
      4'd07: S2 = 4'hA;
      4'd08: S2 = 4'h2;
      4'd09: S2 = 4'h3;
      4'd10: S2 = 4'h8;
      4'd11: S2 = 4'h1;
      4'd12: S2 = 4'h0;
      4'd13: S2 = 4'h7;
      4'd14: S2 = 4'h5;
      4'd15: S2 = 4'h9;
      default: S2 = 4'hX;
    endcase
  end
endfunction


function [3:0] S3( input [3:0] x );
  begin
    case(x)
      4'd00: S3 = 4'h5;
      4'd01: S3 = 4'h8;
      4'd02: S3 = 4'h1;
      4'd03: S3 = 4'hD;
      4'd04: S3 = 4'hA;
      4'd05: S3 = 4'h3;
      4'd06: S3 = 4'h4;
      4'd07: S3 = 4'h2;
      4'd08: S3 = 4'hE;
      4'd09: S3 = 4'hF;
      4'd10: S3 = 4'hC;
      4'd11: S3 = 4'h7;
      4'd12: S3 = 4'h6;
      4'd13: S3 = 4'h0;
      4'd14: S3 = 4'h9;
      4'd15: S3 = 4'hB;
      default: S3 = 4'hX;
    endcase
  end
endfunction


function [3:0] S4( input [3:0] x );
  begin
    case(x)
      4'd00: S4 = 4'h7;
      4'd01: S4 = 4'hD;
      4'd02: S4 = 4'hA;
      4'd03: S4 = 4'h1;
      4'd04: S4 = 4'h0;
      4'd05: S4 = 4'h8;
      4'd06: S4 = 4'h9;
      4'd07: S4 = 4'hF;
      4'd08: S4 = 4'hE;
      4'd09: S4 = 4'h4;
      4'd10: S4 = 4'h6;
      4'd11: S4 = 4'hC;
      4'd12: S4 = 4'hB;
      4'd13: S4 = 4'h2;
      4'd14: S4 = 4'h5;
      4'd15: S4 = 4'h3;
      default: S4 = 4'hX;
    endcase
  end
endfunction


function [3:0] S5( input [3:0] x );
  begin
    case(x)
      4'd00: S5 = 4'h6;
      4'd01: S5 = 4'hC;
      4'd02: S5 = 4'h7;
      4'd03: S5 = 4'h1;
      4'd04: S5 = 4'h5;
      4'd05: S5 = 4'hF;
      4'd06: S5 = 4'hD;
      4'd07: S5 = 4'h8;
      4'd08: S5 = 4'h4;
      4'd09: S5 = 4'hA;
      4'd10: S5 = 4'h9;
      4'd11: S5 = 4'hE;
      4'd12: S5 = 4'h0;
      4'd13: S5 = 4'h3;
      4'd14: S5 = 4'hB;
      4'd15: S5 = 4'h2;
      default: S5 = 4'hX;
    endcase
  end
endfunction


function [3:0] S6( input [3:0] x );
  begin
    case(x)
      4'd00: S6 = 4'h4;
      4'd01: S6 = 4'hB;
      4'd02: S6 = 4'hA;
      4'd03: S6 = 4'h0;
      4'd04: S6 = 4'h7;
      4'd05: S6 = 4'h2;
      4'd06: S6 = 4'h1;
      4'd07: S6 = 4'hD;
      4'd08: S6 = 4'h3;
      4'd09: S6 = 4'h6;
      4'd10: S6 = 4'h8;
      4'd11: S6 = 4'h5;
      4'd12: S6 = 4'h9;
      4'd13: S6 = 4'hC;
      4'd14: S6 = 4'hF;
      4'd15: S6 = 4'hE;
      default: S6 = 4'hX;
    endcase
  end
endfunction


function [3:0] S7( input [3:0] x );
  begin
    case(x)
      4'd00: S7 = 4'hD;
      4'd01: S7 = 4'hB;
      4'd02: S7 = 4'h4;
      4'd03: S7 = 4'h1;
      4'd04: S7 = 4'h3;
      4'd05: S7 = 4'hF;
      4'd06: S7 = 4'h5;
      4'd07: S7 = 4'h9;
      4'd08: S7 = 4'h0;
      4'd09: S7 = 4'hA;
      4'd10: S7 = 4'hE;
      4'd11: S7 = 4'h7;
      4'd12: S7 = 4'h6;
      4'd13: S7 = 4'h8;
      4'd14: S7 = 4'h2;
      4'd15: S7 = 4'hC;
      default: S7 = 4'hX;
    endcase
  end
endfunction


function [3:0] S8( input [3:0] x );
  begin
    case(x)
      4'd00: S8 = 4'h1;
      4'd01: S8 = 4'hF;
      4'd02: S8 = 4'hD;
      4'd03: S8 = 4'h0;
      4'd04: S8 = 4'h5;
      4'd05: S8 = 4'h7;
      4'd06: S8 = 4'hA;
      4'd07: S8 = 4'h4;
      4'd08: S8 = 4'h9;
      4'd09: S8 = 4'h2;
      4'd10: S8 = 4'h3;
      4'd11: S8 = 4'hE;
      4'd12: S8 = 4'h6;
      4'd13: S8 = 4'hB;
      4'd14: S8 = 4'h8;
      4'd15: S8 = 4'hC;
      default: S8 = 4'hX;
    endcase
  end
endfunction

localparam CTRL_IDLE     = 3'h0;
localparam CTRL_INIT     = 3'h1;
localparam CTRL_GENERATE = 3'h2;
localparam CTRL_DONE     = 3'h3;

reg [4 : 0] round_ctr_reg;
reg [4 : 0] round_ctr_new;
reg         round_ctr_rst;
reg         round_ctr_inc;
reg         round_ctr_we;

reg         done_reg;
reg         done_new;
reg         done_we;

reg [63: 0] cdata_reg;
reg [63: 0] cdata_new;
reg         cdata_we;
wire [63:0] cdata_tmp;

reg [2 : 0] state_ctrl_reg;
reg [2 : 0] state_ctrl_new;
reg         state_ctrl_we;

wire [2:0] enc_index = (&round_ctr_reg[4:3]) ? ~round_ctr_reg[2:0] : round_ctr_reg[2:0]; //  cipher key index for encrypt
wire [2:0] dec_index = (|round_ctr_reg[4:3]) ? ~round_ctr_reg[2:0] : round_ctr_reg[2:0]; //  cipher key index for decrypt
wire [2:0] kindex    = mode ? dec_index : enc_index; //  cipher key index

wire [31:0] K [0:7]; // cipher key storage

assign {K[0],K[1],K[2],K[3],K[4],K[5],K[6],K[7]} = key;

reg   [31:0] b, a; // MSB, LSB of input data
wire  [31:0] state_addmod32 = a + K[kindex];  // Adding by module 32
wire  [31:0] state_sbox     = `Sbox(state_addmod32); // S-box replacing
wire  [31:0] state_shift11  = {state_sbox[20:0],state_sbox[31:21]}; // <<11
assign done = done_reg;



//----------------------------------------------------------------
// reg_update
//
// Update functionality for all registers in the core.
// All registers are positive edge triggered with asynchronous
// active low reset. All registers have write enable.
//----------------------------------------------------------------
always @ (posedge clk or posedge rst)
  begin: reg_update
  if (rst)
    begin
      round_ctr_reg    <= 5'h0;
      done_reg         <= 1'b0;
      state_ctrl_reg   <= CTRL_IDLE;
      cdata_reg        <= 64'h0;
    end
  else
    begin
      if (round_ctr_we)
        round_ctr_reg <= round_ctr_new;
      if (done_we)
        done_reg <= done_new;
      if (state_ctrl_we)
        state_ctrl_reg <= state_ctrl_new;
      if (cdata_we)
        cdata_reg <= cdata_new;
    end
end // reg_update

//----------------------------------------------------------------
// round_ctr
//
// The round counter logic with increase and reset.
//----------------------------------------------------------------
always @*
  begin : round_ctr
    round_ctr_new = 5'h0;
    round_ctr_we  = 1'b0;
    if (round_ctr_rst)
      begin
        round_ctr_new = 5'h0;
        round_ctr_we  = 1'b1;
      end
    else if (round_ctr_inc)
      begin
        round_ctr_new = round_ctr_reg + 1'b1;
        round_ctr_we  = 1'b1;
      end
  end

//----------------------------------------------------------------
// cdata_reg
//
// The logic to pivot the ciphertext data
//----------------------------------------------------------------
always @*
  begin : cdata_r
    cdata_new = 64'h0;
    cdata_we  = 1'b0;
    if (state_ctrl_reg == CTRL_DONE)
      begin
        cdata_new = cdata_tmp;
        cdata_we  = 1'b1;
      end
  end
  
//----------------------------------------------------------------
// State_ctrl
//
// The FSM that controls the crypto module.
//----------------------------------------------------------------
always @*
  begin: state_ctrl
    // Default assignments.
    done_new         = 1'b0;
    done_we          = 1'b0;
    round_ctr_rst    = 1'b0;
    round_ctr_inc    = 1'b0;
    state_ctrl_new  = CTRL_IDLE;
    state_ctrl_we   = 1'b0;
    case(state_ctrl_reg)
      CTRL_IDLE:
        begin
          if (load)
            begin
              done_new        = 1'b0;
              done_we         = 1'b1;
              state_ctrl_new = CTRL_INIT;
              state_ctrl_we  = 1'b1;
            end
        end
      CTRL_INIT:
        begin
          round_ctr_rst    = 1'b1;
          state_ctrl_new = CTRL_GENERATE;
          state_ctrl_we  = 1'b1;
        end
      CTRL_GENERATE:
        begin
          round_ctr_inc    = 1'b1;
          if (round_ctr_reg == 31)
            begin
              state_ctrl_new = CTRL_DONE;
              state_ctrl_we  = 1'b1;
            end
        end
      CTRL_DONE:
        begin
          done_new        = 1'b1;
          done_we         = 1'b1;
          state_ctrl_new = CTRL_IDLE;
          state_ctrl_we  = 1'b1;
        end
      default:
        begin
        end
    endcase // case (state_ctrl_reg)
  end // state_ctrl


always @(posedge clk)
  if(rst)
    {b,a} <= {64{1'b0}};
  else if(state_ctrl_reg == CTRL_INIT)
    {b,a} <= pdata;
  else /*if(~&i)*/ begin
    a <= b ^ state_shift11;
    b <= a;
  end

assign cdata_tmp = {a,b};
assign cdata = cdata_reg;
endmodule