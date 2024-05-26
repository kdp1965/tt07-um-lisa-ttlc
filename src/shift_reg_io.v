/*
==============================================================================
shift_reg_io.v:  Tiny Tapeout Logic Controller (MC14500 based) 
                 shift-register based external I/O interface.

Copyright 2024 by Ken Pettit <pettitkd@gmail.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

==============================================================================
*/
module shift_reg_io
(
   input  wire          clk,                 // System clock
   input  wire          rst_n,               // System reset
   input  wire [2:0]    data_in,             // Serial data inputs
   input  wire [47:0]   output_buffer,       // Input to the module, drives the serial output
   input  wire [1:0]    clk_div,             // Clock division factor control (00: /2, 01: /4, 10: /8, 11: /16)
   input  wire          start,               // Start the I/O transfer
   input  wire [1:0]    input_depth,
   input  wire [1:0]    output_depth,
   output reg           complete,            // Indicates I/O transfer complete
   output reg  [2:0]    data_out,            // Serial data outputs
   output reg           latch,               // Latch signal for external register control
   output reg           shift_clk,           // Shift clock output for external shift registers
   output wire [47:0]   input_buffer         // Output from the module, captures the serial input
);

   // Parameters
   localparam integer DATA_WIDTH   = 48;    // Total data width for input and output
   localparam integer SEG_WIDTH    = 16;     // Width of each data segment
//   localparam integer SHIFT_CYCLES = 32;     // Number of cycles to shift each bit segment
   localparam [0:0]   IDLE         = 1'd0,
                      SHIFT_IO     = 1'd1;
                      
   wire [1:0]              max_depth;
   wire [47:0]             output_comb;

   // Internal Registers for input buffer
   reg  [DATA_WIDTH-1:0]   input_buffer_reg;
   reg  [3:0]              clk_count;        // Counter for clock division
   wire [3:0]              max_count;        // Maximum count value based on clk_div
   reg  [4:0]              shift_count;      // Count for the number of shifts performed
   reg  [0:0]              state;
   reg  [4:0]              cycles;

   assign max_depth = input_depth > output_depth ? input_depth : output_depth;
   assign input_buffer = input_buffer_reg; // Assign the internal register to the output
   assign max_count = (clk_div == 2'b00) ? 1 :
                      (clk_div == 2'b01) ? 3 :
                      (clk_div == 2'b10) ? 7 :
                      15;                 // Default to 15 for clk_div == 2'b11
   assign output_comb = {output_buffer};

   always @*
   begin
      case (max_depth)
      2'h0: cycles = 5'h8;
      2'h1: cycles = 5'h16;
      2'h2: cycles = 5'h16;
      2'h3: cycles = 5'h16;
      endcase
   end

   // State Machine for controlling the process
   // Clock and shift logic
   always @(posedge clk)
   begin
      if (~rst_n)
      begin
         input_buffer_reg  <= 48'h0;
         clk_count         <= 4'h0;
         shift_clk         <= 1'b0;
         shift_count       <= 5'h0;              // Reset shift_count
         state             <= IDLE;
         latch             <= 1'b1;
         complete          <= 1'b0;
      end
      else
      begin
         // Clock division for shift_clk
         if (clk_count >= max_count)
         begin
            clk_count <= 4'h0;
            shift_clk <= ~shift_clk;  // Toggle shift clock
         end
         else
            clk_count <= clk_count + 1;
         
         // State machine for shifting data
         case (state)
            IDLE:
            begin
               if (start)
               begin
                  complete    <= 1'b0;
                  shift_count <= 5'h0;    // Reset shift count at start of operation
                  latch       <= 1'b0;       // Pulse to capture inputs before shifting
                  state       <= SHIFT_IO;
               end
            end

            SHIFT_IO:
            begin
               if (shift_clk && clk_count == 0)  // Operate on rising edge of shift_clk
               begin  
                  // Test if more input data to shift
                  if (shift_count[4:3] <= {1'b0, input_depth[0]})
                  begin
                     // Shifting input data in, and preparing output data
                     input_buffer_reg[6'(shift_count)] <= data_in[0];
                     input_buffer_reg[6'(shift_count) + 6'(SEG_WIDTH)] <= data_in[1];
                     input_buffer_reg[6'(shift_count) + 6'(2*SEG_WIDTH)] <= data_in[2];
                     //input_buffer_reg[6'(shift_count) + 6'(3*SEG_WIDTH)] <= data_in[3];
                  end

                  // Test if more output data to shift
                  if (shift_count[4:3] <= {1'b0, output_depth[0]})
                  begin
                     data_out[0] <= output_comb[6'(shift_count)];
                     data_out[1] <= output_comb[6'(shift_count) + 6'(SEG_WIDTH)];
                     data_out[2] <= output_comb[6'(shift_count) + 6'(2*SEG_WIDTH)];
//                     data_out[3] <= output_comb[6'(shift_count) + 6'(3*SEG_WIDTH)];
                  end
                  shift_count <= shift_count + 1;
               end
               if (shift_count == {{1'b0, output_depth[0]} + 2'h1, 3'h0})  // Check if we've shifted all bits
                  latch    <= 1'b0;     // Pulse to latch the outputs after the final shifting
               else
                  latch    <= 1'b1;

               if (shift_count == 5'(cycles))
               begin
                  complete <= 1'b1;
                  state    <= IDLE;
               end
            end

         endcase
      end
   end
endmodule

