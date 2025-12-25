module CoPro6502(
	// Host
	input         h_clk,
	input         h_cs_b,
	input         h_rdnw,
	input   [2:0] h_addr,
	input   [7:0] h_data_in,
	output  [7:0] h_data_out,
	input         h_rst_b,
	output        h_irq_b,

	// Parasite Clock (32 MHz)
	input         clk_cpu,
	input         cpu_clken,

	// External RAM
	output [15:0] ram_addr,
	output  [7:0] ram_data_in,
	input   [7:0] ram_data_out,
	output        ram_wr,

	// Test signals for debugging
	output  [7:0] test
);

//-----------------------------------------------
// clock and reset signals
//-----------------------------------------------

	reg  bootmode;
	wire RSTn;
	reg  RSTn_sync;
	reg  [8:0] reset_counter;

//-----------------------------------------------
// parasite signals
//-----------------------------------------------

	wire p_cs_b;
	wire [7:0] p_data_out;

//-----------------------------------------------
// ram/rom signals
//-----------------------------------------------

	wire ram_cs_b;
	wire rom_cs_b;
	wire [7:0] rom_data_out;

//-----------------------------------------------
// cpu signals
//-----------------------------------------------

	wire cpu_R_W_n;
	wire [23:0] cpu_addr;
	wire [23:0] cpu_addr_us;
	wire  [7:0] cpu_din;
	wire  [7:0] cpu_dout;
	wire  [7:0] cpu_dout_us;
	wire cpu_IRQ_n;
	wire cpu_NMI_n;
	reg  cpu_IRQ_n_sync;
	reg  cpu_NMI_n_sync;
	wire sync;

//-------------------------------------------------------------------
// instantiated components
//---------------------------------------------------------------------

	tuberom_65c102 inst_tuberom (
		.CLK    ( clk_cpu ),
		.ADDR   ( cpu_addr[10:0] ),
		.DATA   ( rom_data_out )
	);

	r65c02 inst_r65c02 (
		.reset    ( RSTn_sync ),
		.clk      ( clk_cpu ),
		.enable   ( cpu_clken ),
		.nmi_n    ( cpu_NMI_n_sync ),
		.irq_n    ( cpu_IRQ_n_sync ),
		.di       ( cpu_din ),
		.do       ( cpu_dout ),
		.addr     ( cpu_addr[15:0] ),
		.nwe      ( cpu_R_W_n ),
		.sync     ( sync ),
		.sync_irq ( )
	);

	tube inst_tube (
		.h_addr     ( h_addr ),
		.h_cs_b     ( h_cs_b ),
		.h_data_in  ( h_data_in ),
		.h_data_out ( h_data_out ),
		.h_phi2     ( h_clk ),
		.h_rdnw     ( h_rdnw ),
		.h_rst_b    ( h_rst_b ),
		.h_irq_b    ( h_irq_b ),
		.p_addr     ( cpu_addr[2:0] ),
		.p_cs_b     ( ~(~p_cs_b & cpu_clken) ),
		.p_data_in  ( cpu_dout ),
		.p_data_out ( p_data_out ),
		.p_rdnw     ( cpu_R_W_n ),
		.p_phi2     ( clk_cpu ),
		.p_rst_b    ( RSTn ),
		.p_nmi_b    ( cpu_NMI_n ),
		.p_irq_b    ( cpu_IRQ_n )
	);


	assign p_cs_b = cpu_addr[15:3] != 13'b1111111011111;

	assign rom_cs_b = ~(cpu_addr[15:11] == 5'b11111 & cpu_R_W_n & bootmode);

	assign ram_cs_b = ~(p_cs_b & rom_cs_b);

	assign ram_wr = ~ram_cs_b & ~cpu_R_W_n;

	assign ram_data_in = cpu_dout;

	assign ram_addr = cpu_addr[15:0];

	assign cpu_din =
	  !p_cs_b ? p_data_out :
	  !rom_cs_b ? rom_data_out :
	  !ram_cs_b ? ram_data_out :
	  8'hf1;

//------------------------------------------------------
// boot mode generator
//------------------------------------------------------
	always @(posedge clk_cpu) begin : boot_gen
		if (!RSTn_sync)
			bootmode <= 1;
		else if (!p_cs_b)
			bootmode <= 0;
	end

//------------------------------------------------------
// power up reset
//------------------------------------------------------
	always @(posedge clk_cpu) begin : reset_gen
		if (!reset_counter[8])
			reset_counter <= reset_counter + 1'd1;
		RSTn_sync <= RSTn & reset_counter[8];
	end

//------------------------------------------------------
// interrupt synchronization
//------------------------------------------------------
	always @(posedge clk_cpu) begin : sync_gen
		if (!RSTn_sync) begin
			cpu_NMI_n_sync <= 1;
			cpu_IRQ_n_sync <= 1;
		end else if (cpu_clken) begin
			cpu_NMI_n_sync <= cpu_NMI_n;
			cpu_IRQ_n_sync <= cpu_IRQ_n;
		end
	end

assign test = {RSTn, RSTn_sync, h_rst_b, cpu_NMI_n_sync, cpu_IRQ_n_sync, bootmode, 2'b00};

endmodule
