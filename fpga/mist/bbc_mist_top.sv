`timescale 1ns / 1ps
// bbc_mist_top.v
module bbc_mist_top(

  // clock inputs
  input wire [1:0] 	CLOCK_27, // 27 MHz
  
  // LED outputs
  output wire	LED, // LED Yellow
  
  // VGA
  output wire	VGA_HS, // VGA H_SYNC
  output wire	VGA_VS, // VGA V_SYNC
  output wire [5:0] 	VGA_R, // VGA Red[5:0]
  output wire [5:0] 	VGA_G, // VGA Green[5:0]
  output wire [5:0] 	VGA_B, // VGA Blue[5:0];
	
	// AUDIO
	output wire 		AUDIO_L, // sigma-delta DAC output left
	output wire 		AUDIO_R, // sigma-delta DAC output right
	
	// SDRAM
   inout [15:0]    SDRAM_DQ,       // SDRAM Data bus 16 Bits
   output [12:0]   SDRAM_A,        // SDRAM Address bus 13 Bits
   output          SDRAM_DQML,     // SDRAM Low-byte Data Mask
   output          SDRAM_DQMH,     // SDRAM High-byte Data Mask
   output          SDRAM_nWE,      // SDRAM Write Enable
   output          SDRAM_nCAS,     // SDRAM Column Address Strobe
   output          SDRAM_nRAS,     // SDRAM Row Address Strobe
   output          SDRAM_nCS,      // SDRAM Chip Select
   output [1:0]    SDRAM_BA,       // SDRAM Bank Address
   output          SDRAM_CLK,      // SDRAM Clock
   output          SDRAM_CKE,      // SDRAM Clock Enable
	
  
  // SPI
  output         SPI_DO,
  input          SPI_DI,
  input          SPI_SCK,
  input          SPI_SS2,    // data_io
  input          SPI_SS3,    // OSD
  input          CONF_DATA0  // SPI_SS for user_io
);

assign LED = ~loader_active;

// the configuration string is returned to the io controller to allow
// it to control the menu on the OSD 
parameter CONF_STR = {
        "BBC;ROM;",
        "S1U,SSDDSD,Mount Disk;",
        "O12,Scanlines,Off,25%,50%,75%;",
        "O3,Joystick Swap,Off,On;",
        "O4,Mode,Model B,Master;",
        "O5,ROM mapping,High,Low;",
        "O6,Auto boot,Off,On;",
        "R64,Save CMOS;",
        "T0,Reset;"
};

wire [1:0] scanlines = status[2:1];
wire       joyswap = status[3];
wire       model = status[4];
wire       rommap = status[5];
wire       autoboot = status[6];

// generated clocks
wire clk_48m /* synthesis keep */ ;

wire pll_ready;

// core's raw video 
wire 			core_r, core_g, core_b, core_hs, core_vs;   
wire			core_clken;

// memory bus signals.
wire [15:0] mem_adr;
wire [7:0]  mem_romsel;
wire        shadow_ram;
wire        shadow_vid;
wire        mem_acc_y;

wire [7:0]  mem_di;
wire [7:0]  rom_do;
wire [7:0]  ram_do;

wire [7:0]  mem_do;
wire        mem_we;
wire        mem_sync;
wire        phi0;

// core's raw audio 
wire [15:0]	coreaud_l, coreaud_r;

// user io
wire [7:0] status;
wire [1:0] buttons;
wire [1:0] switches;

wire        ps2_clk;
wire        ps2_dat;

// the top file should generate the correct clocks for the machine

assign SDRAM_CLK = clk_48m;

clockgen CLOCKS(
	.inclk0	(CLOCK_27[0]),
	.c0		(clk_48m),
	.locked	(pll_ready)  // pll locked output
);

// conections between user_io (implementing the SPI communication 
// to the io controller) and the legacy 
wire sd_busy;
wire [31:0] sd_lba = sd_busy ? sd_lba_mmfs : sd_lba_fdc;
wire [1:0] sd_rd;
wire [1:0] sd_wr;
wire sd_ack;
wire sd_conf;
wire sd_sdhc; 
wire [7:0] sd_dout;
wire sd_dout_strobe;
wire [7:0] sd_din = sd_busy ? sd_din_mmfs : sd_din_fdc;
wire [8:0] sd_buff_addr;
wire sd_ack_conf;
wire [1:0] img_mounted;
wire [31:0] img_size;

wire [7:0] joystick_0;
wire [7:0] joystick_1;
wire [15:0] joystick_analog_0;
wire [15:0] joystick_analog_1;

wire scandoubler_disable;
wire ypbpr;
wire no_csync;
wire [63:0] RTC;

user_io #(.STRLEN($size(CONF_STR)>>3)) user_io(
	.conf_str      ( CONF_STR       ),
	.clk_sys       ( clk_48m        ),
	.clk_sd        ( clk_48m        ),

	// the spi interface
	.SPI_CLK        ( SPI_SCK       ),
	.SPI_SS_IO      ( CONF_DATA0    ),
	.SPI_MISO       ( SPI_DO        ),   // tristate handling inside user_io
	.SPI_MOSI       ( SPI_DI        ),
	
	.joystick_0        ( joystick_0 ),
	.joystick_1        ( joystick_1 ),
	.joystick_analog_0 ( joystick_analog_0 ),
	.joystick_analog_1 ( joystick_analog_1 ),

	.status         ( status        ),
	.switches       ( switches      ),
	.buttons        ( buttons       ),
	.scandoubler_disable ( scandoubler_disable ),
	.ypbpr          ( ypbpr         ),
	.no_csync       ( no_csync      ),
	.rtc            ( RTC           ),

   // interface to embedded legacy sd card wrapper
	.sd_lba     	  ( sd_lba        ),
	.sd_rd      	  ( sd_rd         ),
	.sd_wr      	  ( sd_wr         ),
	.sd_ack     	  ( sd_ack        ),
	.sd_conf    	  ( sd_conf       ),
	.sd_sdhc    	  ( sd_sdhc       ),
	.sd_dout    	  ( sd_dout       ),
	.sd_dout_strobe ( sd_dout_strobe),
	.sd_din     	  ( sd_din        ),
	.sd_buff_addr   ( sd_buff_addr  ),
	.sd_ack_conf    ( sd_ack_conf   ),

	.img_mounted    ( img_mounted   ),
	.img_size       ( img_size      ),

	.ps2_kbd_clk	  ( ps2_clk       ), 
	.ps2_kbd_data	  ( ps2_dat       )
);

// wire the sd card to the user port
wire sd_sck;
wire sd_cs;
wire sd_sdi;
wire sd_sdo;
wire [31:0] sd_lba_mmfs;
wire  [7:0] sd_din_mmfs;

sd_card sd_card (
	// connection to io controller
	.clk_sys      ( clk_48m        ),
	.sd_lba       ( sd_lba_mmfs    ),
	.sd_rd        ( sd_rd[0]       ),
	.sd_wr        ( sd_wr[0]       ),
	.sd_ack       ( sd_ack         ),
	.sd_ack_conf  ( sd_ack_conf    ),
	.sd_conf      ( sd_conf        ),
	.sd_sdhc      ( sd_sdhc        ),
	.sd_buff_dout ( sd_dout        ),
	.sd_buff_wr   ( sd_dout_strobe ),
	.sd_buff_din  ( sd_din_mmfs    ),
	.sd_buff_addr ( sd_buff_addr   ),
	.img_mounted  ( img_mounted[0] ),
	.img_size     ( img_size       ),
	.allow_sdhc   ( 1'b1           ),
	.sd_busy      ( sd_busy        ),
 
	// connection to local CPU
	.sd_cs        ( sd_cs          ),
	.sd_sck       ( sd_sck         ),
	.sd_sdi       ( sd_sdi         ),
	.sd_sdo       ( sd_sdo         )
);

// data loading 
wire        loader_active, upload_active;
wire        loader_we, ioctl_we;
wire [24:0]	loader_addr, ioctl_addr;
wire  [7:0] loader_data, ioctl_data, ioctl_din;
wire  [7:0] ioctl_index;

always @(posedge clk_48m) begin
	reg we_int = 0;

	if (mem_sync) begin
		we_int <= 0;
		loader_we <= we_int;
		if (we_int) begin
			loader_addr <= ioctl_addr + (ioctl_index == 0 ? 20'h80000 : { 7'b0000001, 4'ha, 14'h0 });
			loader_data <= ioctl_data;
		end
	end

	if (ioctl_we && loader_active && ioctl_index != 8'hff) we_int <= 1;
end
/*
ROM structure:
Model B:
00000 - 03FFF OS12 (Model B)
04000 - 07FFF MOS  (Master)
08000 - 0FFFF empty
10000 - 1FFFF Pages C-D-E-F (or 0-1-2-3)
 BASIC, empty, MMFS, empty
Master:
20000 - 2FFFF Pages 0-1-2-3
 empty, empty, ADFS1-57, MAMMFS
30000 - 3FFFF empty (RAM area)
40000 - 5FFFF Pages 8-9-A-B-C-D-E-F
 empty, DFS, VIEWSHT, EDIT, BASIC4, ADFS, VIEW, TERMINAL
*/

data_io DATA_IO (
	.clk_sys    ( clk_48m ),
	.SPI_SCK    ( SPI_SCK ),
	.SPI_SS2    ( SPI_SS2 ),
	.SPI_DI     ( SPI_DI  ),
	.SPI_DO     ( SPI_DO  ),

	.ioctl_download ( loader_active ),
	.ioctl_upload   ( upload_active  ),
	.ioctl_index( ioctl_index  ),

   // ram interface
	.ioctl_wr   ( ioctl_we     ),
	.ioctl_addr ( ioctl_addr   ),
	.ioctl_dout ( ioctl_data   ),
	.ioctl_din  ( ioctl_din    )
);

wire [7:0] user_via_pb_out;
wire user_via_cb1_in;
wire user_via_cb2_in;

// reset core whenever the user changes the rom mapping
reg last_rom_map, last_model;
reg [11:0] rom_map_counter = 12'h0;
always @(posedge clk_48m) begin
	last_rom_map <= rommap;
	last_model <= model;

	if(last_rom_map != rommap || last_model != model)
		rom_map_counter <= 12'hfff;
	else if(rom_map_counter != 0)
		rom_map_counter <= rom_map_counter - 12'd1;
end

wire rom_remap_reset = (rom_map_counter != 0);

// the bbc is being reset of the pll isn't stable, if the ram isn't ready,
// of the arm boots or if the user selects reset from the osd or of the user
// presses the "core" button or the io controller uploads a rom
wire reset_in = ~pll_ready || ~sdram_ready || status[0] ||
		buttons[1] || loader_active || rom_remap_reset;

// synchronize reset with memory state machine
reg reset;
always @(posedge clk_48m)
	if (mem_sync) reset <= reset_in;

// the autoboot feature simply works by pressing shift for 2 seconds after 
// the bbc has been reset
wire autoboot_shift = autoboot && (autoboot_counter != 0 );
reg [24:0] autoboot_counter;
always @(posedge clk_48m) begin
	if(reset) 
		autoboot_counter <= 25'd32000000;
	else if(autoboot_counter != 0)
		autoboot_counter <= autoboot_counter - 25'd1;
end

wire [31:0] sd_lba_fdc;
wire  [7:0] sd_din_fdc;

wire img_ds = ioctl_index[7:6] == 1;

bbc BBC(

	.CLK48M_I   ( clk_48m       ),
	.RESET_I    ( reset         ),

	.MODEL_I    ( model         ),

	.HSYNC      ( core_hs       ),
	.VSYNC      ( core_vs       ),

	.VIDEO_CLKEN( core_clken    ),

	.VIDEO_R    ( core_r        ),
	.VIDEO_G    ( core_g        ),
	.VIDEO_B    ( core_b        ),
	.VIDEO_DE   ( video_de      ),

	.MEM_ADR    ( mem_adr       ),
	.MEM_WE     ( mem_we        ),
	.MEM_DO     ( mem_do        ),
	.MEM_DI     ( mem_di        ),
	.MEM_SYNC   ( mem_sync      ),
	.ROMSEL     ( mem_romsel    ),
	.ACC_Y      ( mem_acc_y     ),
	.SHADOW_RAM ( shadow_ram    ),
	.SHADOW_VID ( shadow_vid    ),
	.PHI0       ( phi0          ),

	.SHIFT      ( autoboot_shift ),

	.SDCLK      (sd_sck         ),
	.SDSS       (sd_cs          ),
	.SDMISO     (sd_sdo         ),
	.SDMOSI     (sd_sdi         ),

	.joy_but    ( { joystick_1[4], joystick_0[4] } ),
	.joy0_axis0 ( joyswap ? joystick_analog_1[15:8] : joystick_analog_0[15:8] ),
	.joy0_axis1 ( joyswap ? joystick_analog_1[ 7:0] : joystick_analog_0[ 7:0] ),
	.joy1_axis0 ( joyswap ? joystick_analog_0[15:8] : joystick_analog_1[15:8] ),
	.joy1_axis1 ( joyswap ? joystick_analog_0[15:8] : joystick_analog_1[ 7:0] ),

	.DIP_SWITCH ( 8'b00000000 ),

	.PS2_CLK	( ps2_clk       ),
	.PS2_DAT	( ps2_dat       ),

	.AUDIO_L	( coreaud_l     ),
	.AUDIO_R	( coreaud_r     ),

	// FDC connection
	.img_mounted    ( img_mounted[1] ),
	.img_size       ( img_size       ),
	.img_ds         ( img_ds         ),
	.sd_lba         ( sd_lba_fdc     ),
	.sd_rd          ( sd_rd[1]       ),
	.sd_wr          ( sd_wr[1]       ),
	.sd_ack         ( sd_ack         ),
	.sd_buff_addr   ( sd_buff_addr   ),
	.sd_dout        ( sd_dout        ),
	.sd_din         ( sd_din_fdc     ),
	.sd_dout_strobe ( sd_dout_strobe ),
	// CMOS RAM
	.RTC            ( RTC            ),
	.cmos_addr      ( ioctl_addr[6:0]),
	.cmos_we        ( ioctl_we & loader_active & ioctl_index == 8'hff ),
	.cmos_di        ( ioctl_data     ),
	.cmos_do        ( ioctl_din      )
);

assign SDRAM_CKE = 1'b1;
wire sdram_ready;

// CPU address mapping
wire cpu_ram = (mem_adr[15] == 1'b0);
wire mos_rom = (mem_adr[15:14] == 2'b11);
wire sideways = (mem_adr[15:14] == 2'b10);
wire mos_ram = sideways & (mem_adr[13:12] == 2'b00) & mem_romsel[7];
wire filing_ram = (mem_adr[15:13] == 3'b110) & mem_acc_y;

// map 64k sideways ram to bank 4,5,6 and 7
wire sideways_ram = sideways & (mem_romsel[3:2] == 2'b01);

// Master: pages 0-3, 8-F
// Model B: rommap is '1' of low mapping is selected in the menu
wire sideways_rom = sideways &
                    model  ? (mem_romsel[3:2] == 2'b00 || mem_romsel[3]) :
                             rommap?(mem_romsel[3:2] == 2'b00):(mem_romsel[3:2] == 2'b11);

/*
 SDRAM map
 00000-07FFF Main RAM
 08000-08FFF MOS private RAM (Master)
 0A000-0BFFF Filing system RAM (Master)
 13000-17FFF Shadow RAM (Master)
 40000-7FFFF sideways ram access
 80000-DFFFF ROMs
*/

wire [24:0] sdram_adr =
	loader_active ? loader_addr:
	~phi0 ? { shadow_vid, mem_adr }:                    // video access
	(cpu_ram | mos_ram) ? { shadow_ram, mem_adr }:      // ordinary RAM access: 0000-7FFF + 8000-8FFF (MOS Private RAM)
	filing_ram ? { 3'b101, mem_adr[12:0] }:             // Filing system RAM: A000-BFFF
	mos_rom ? { 4'h8, 1'b0, model, mem_adr[13:0] }:     // OS12 or MOS: 80000-87FFF
	(sideways_rom && ~model) ? { 4'h9, mem_romsel[1:0], mem_adr[13:0] }: // Model B ROMs: 9xxxx
	(sideways_rom &&  model) ? { 4'hA + mem_romsel[3:2], mem_romsel[1:0], mem_adr[13:0] }: // Master ROMs: A0000-DFFFF
	{ 1'b1, mem_romsel[3:0], mem_adr[13:0] };          // sideways RAM access (page 4-5-6-7)

wire sdram_we = loader_active?loader_we:(mem_we && (cpu_ram || sideways_ram || mos_ram || filing_ram));

wire [7:0] sdram_di = 
	loader_active?loader_data:mem_do;

wire video_de;

sdram sdram (
	// interface to the MT48LC16M16 chip
	.sd_data        ( SDRAM_DQ                 ),
	.sd_addr        ( SDRAM_A                  ),
	.sd_dqm         ( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs          ( SDRAM_nCS                ),
	.sd_ba          ( SDRAM_BA                 ),
	.sd_we          ( SDRAM_nWE                ),
	.sd_ras         ( SDRAM_nRAS               ),
	.sd_cas         ( SDRAM_nCAS               ),

	// system interface
	.clk            ( clk_48m                  ),
	.sync           ( mem_sync                 ),
	.init           ( !pll_ready               ),
	.ready          ( sdram_ready              ),

	// cpu/video interface
	.cpu_di         ( sdram_di                 ),
	.cpu_adr        ( sdram_adr                ),
	.cpu_we         ( sdram_we                 ),
	.cpu_do         ( ram_do                   ),

	.vid_blnk       ( !video_de                ) // for refresh
);

assign mem_di = ram_do;

audio	AUDIO	(
	.clk         ( clk_48m    ),
	.rst         ( ~pll_ready ),
	.audio_data_l( coreaud_l  ),
	.audio_data_r( coreaud_r  ),
	.audio_l     ( AUDIO_L    ),
	.audio_r     ( AUDIO_R    )
);

mist_video #(.COLOR_DEPTH(1), .SD_HCNT_WIDTH(11), .SYNC_AND(1)) mist_video (
	.clk_sys     ( clk_48m    ),

	// OSD SPI interface
	.SPI_SCK     ( SPI_SCK    ),
	.SPI_SS3     ( SPI_SS3    ),
	.SPI_DI      ( SPI_DI     ),

	// scanlines (00-none 01-25% 10-50% 11-75%)
	.scanlines   ( scanlines  ),

	// non-scandoubled pixel clock divider 0 - clk_sys/4, 1 - clk_sys/2
	.ce_divider  ( 3'd1       ),

	// 0 = HVSync 31KHz, 1 = CSync 15KHz
	.scandoubler_disable ( scandoubler_disable ),
	// disable csync without scandoubler
	.no_csync    ( no_csync   ),
	// YPbPr always uses composite sync
	.ypbpr       ( ypbpr      ),
	// Rotate OSD [0] - rotate [1] - left or right
	.rotate      ( 2'b00      ),
	// composite-like blending
	.blend       ( 1'b0       ),

	// video in
	.R           ( core_r     ),
	.G           ( core_g     ),
	.B           ( core_b     ),

	.HSync       ( ~core_hs   ),
	.VSync       ( ~core_vs   ),

	// MiST video output signals
	.VGA_R       ( VGA_R      ),
	.VGA_G       ( VGA_G      ),
	.VGA_B       ( VGA_B      ),
	.VGA_VS      ( VGA_VS     ),
	.VGA_HS      ( VGA_HS     )
);

endmodule // bbc_mist_top
