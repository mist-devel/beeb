`timescale 1ns / 1ps

// Partial SERPROC implementation
// No cassette parts

module serproc (
	input CLOCK,
	input nRESET,
	input CS,
	input [7:0] DI,

	output RX_CLK_EN,
	output TX_CLK_EN,
	output CTS_N,
	output DCD_N,
	input RTS_N,
	output RX,
	input TX,

	input RS232_CTS,
	output RS232_RTS,
	input RS232_RX,
	output RS232_TX,
	output CASS_MOTOR
);

reg   [2:0] rx_clk, tx_clk;
reg         motor_on, serial_en;

assign CASS_MOTOR = motor_on;

always @(posedge CLOCK) begin
	if (!nRESET)
		{motor_on, serial_en, rx_clk, tx_clk} <= 0;
	else if (CS) 
		{motor_on, serial_en, rx_clk, tx_clk} <= DI;
end

assign DCD_N     = 0;
assign CTS_N     = serial_en ? RS232_CTS : 1'b0;
assign RX        = serial_en ? RS232_RX  : 1'b1;
assign RS232_TX  = serial_en ? TX : 1'b1;
assign RS232_RTS = serial_en ? RTS_N : 1'b1;

serproc_clockdiv rx_clockdiv(CLOCK, !nRESET, rx_clk, RX_CLK_EN);
serproc_clockdiv tx_clockdiv(CLOCK, !nRESET, tx_clk, TX_CLK_EN);

endmodule

// assuming a 48 MHz clock
module serproc_clockdiv(
	input  clk,
	input  reset,
	input  [2:0] setting,
	output reg clk_en
);

reg [15:0] cnt, divider;
always @(posedge clk) begin
	clk_en <= 0;
	if (reset)
		cnt <= 0;
	else begin
		cnt <= cnt + 1'd1;
		if (cnt == divider) begin
			clk_en <= 1;
			cnt <= 0;
		end
	end
end

always @(*)
	case ({setting[0], setting[1], setting[2]})
		0: divider = 38; // 19200 baud
		1: divider = 77; // 9600 baud
		2: divider = 155; // 4800 baud
		3: divider = 266; // 2400 baud
		4: divider = 624; // 1200 baud
		5: divider = 2499; // 300 baud
		6: divider = 4999; // 150 baud
		7: divider = 9999; // 75 baud
	endcase

endmodule