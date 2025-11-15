	`timescale 1ns / 1ps
/*
Copyright (c) 2012, Stephen J Leary
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL STEPHEN J LEARY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
module clocks(

		input clk_48m,
		
		input reset_n,
		input mhz1_enable,
		
		output wire mhz6_clken,
		output wire mhz4_clken,
		output wire mhz2_clken,
		output wire mhz1_clken,
		
		output wire cpu_cycle,
		output wire cpu_clken,
		output wire cpu_phi0,
		
		output wire vid_clken,
		output wire ttxt_clken,
		output wire ttxt_clkenx2,
		
		output wire tube_clken
    );

reg     [5:0] clken_counter; 
reg     [1:0] cpu_cycle_mask; 

//  Clock enable generation - 48 MHz clock split into 48 cycles
//  CPU is on 0 and 24 (but can be masked by 1 MHz bus accesses)
//  Video is on all third cycles (16 MHz)
//  1 MHz cycles are on cycle 47 (1 MHz)
assign vid_clken =
	clken_counter == 1 ||
	clken_counter == 4 ||
	clken_counter == 7 ||
	clken_counter == 10 ||
	clken_counter == 13 ||
	clken_counter == 16 ||
	clken_counter == 19 ||
	clken_counter == 22 ||
	clken_counter == 25 ||
	clken_counter == 28 ||
	clken_counter == 31 ||
	clken_counter == 34 ||
	clken_counter == 37 ||
	clken_counter == 40 ||
	clken_counter == 43 ||
	clken_counter == 46;

assign mhz6_clken = clken_counter == 15 || clken_counter == 31 || clken_counter == 47;
//  1,3,5...
assign mhz4_clken = clken_counter == 11 || clken_counter == 23 || clken_counter == 35 || clken_counter == 47;
//  15/31
assign mhz2_clken = clken_counter == 23 || clken_counter == 47;
// 23/47
assign tube_clken = clken_counter == 1 || clken_counter == 25;
// 1/25
assign mhz1_clken = clken_counter == 47;
// 47
assign cpu_cycle = clken_counter == 0 || clken_counter == 24;
// 0/24
assign cpu_clken = cpu_cycle & ~cpu_cycle_mask[1] & ~cpu_cycle_mask[0];

assign cpu_phi0 = clken_counter == 0 || clken_counter >= 36 || (clken_counter > 12 && clken_counter <= 24);

always @(posedge clk_48m)
	begin : clk_gen
		if (clken_counter == 47)
			clken_counter <= 0;
		else
			clken_counter <= clken_counter + 1'd1;
	end


always @(posedge clk_48m)
   begin : cycle_stretch
   if (reset_n === 1'b 0) begin
      cpu_cycle_mask <= 2'b 00;   
      end
   else if (mhz2_clken === 1'b 1 ) begin
			if ((mhz1_enable === 1'b 1) && (cpu_cycle_mask === 2'b 00)) begin
				//  Block CPU cycles until 1 MHz cycle has completed
				if (mhz1_clken === 1'b 0) begin
					cpu_cycle_mask <= 2'b 01;   
				end else begin
					cpu_cycle_mask <= 2'b 10;
				end
			end
			if (cpu_cycle_mask !== 2'b 00) begin
				cpu_cycle_mask <= cpu_cycle_mask - 2'b 01;
			end
		end
   end

//  6 MHz clock enable for SAA5050
assign ttxt_clken = clken_counter[2:0] == 0; 
assign ttxt_clkenx2 = clken_counter[1:0] == 0;

endmodule
