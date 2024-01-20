module sim_video_stream_new;

    // Parameters
    parameter integer N_MIPI_LANES = 2;
    parameter integer SERDES_DATA_WIDTH = 8;
    parameter integer ADD_DEBUG_OVERLAY = 0; // We don't add overlay in simulation

    // Interface Signals
    logic sys_clk_p;  // AD12 SYSCLK_P
    logic sys_clk_n;  // AD11 SYSCLK_N
    logic sys_rst;
	logic i2c3_cam_clk;
	logic monitor_1p2v;
	logic i2c2_cam_clk;
	wire i2c3_cam_dat;
	logic switch_data_lanes_vadj;
	logic switch_clock_lanes_vadj;
	wire i2c_cam_dat;
	wire i2c_cam_clk;
	logic monitor_1p8v;
	logic cam0_rst;
	logic cam0_pwr;
	logic cam0_mclk;
	logic monitor_2p8v;
	logic monitor_3p3v;
	wire i2c2_cam_dat;
	logic en_mipi_out_data_vadj;
	logic en_mipi_out_clock_vadj;
	logic enn_jt_gpio_input;
	logic gpio17_out;
	logic gpio9_out;

	logic hs_c_d1_p;
	logic hs_c_d1_n;
	logic hs_c_d0_p;
	logic hs_c_d0_n;
	logic lp_c_d1_p;
	logic lp_c_d1_n;
	logic lp_c_d0_p;
	logic lp_c_d0_n;
	logic lp_c_clk_p;
	logic lp_c_clk_n;

	logic hs_d_d1_p;
	logic hs_d_d1_n;
	logic hs_d_d0_p;
	logic hs_d_d0_n;
	logic hs_d_clk_p;
	logic hs_d_clk_n;
	logic hs_c_clk_p;
	logic hs_c_clk_n;

	logic lp_d_d1_p;
	logic lp_d_d1_n;
	logic lp_d_d0_p;
	logic lp_d_d0_n;
	logic lp_d_clk_p;
	logic lp_d_clk_n;

	logic leds_debug;
	logic dbg_io_1;
	logic dbg_io_2;
	logic dbg_io_3;
	logic GPIO_SW_N;
	logic GPIO_SW_E;
	logic GPIO_SW_S;
	logic GPIO_SW_W;
	logic GPIO_SW_C;

	// RX sigs
	 logic       MIPI_CLK;
   logic [15:0] PIXEL_NUM;
   logic [15:0] LINE_NUM;
   logic       DETECT;
   logic       FS;
   logic       FE;
   logic [7:0]  L0;
   logic [7:0]  L1;
   logic [15:0] DET_D0;
   logic [15:0] DET_D1;

   logic [7:0]  MIPI_LANE0;
   logic [7:0]  MIPI_LANE1;

   logic       MIPI_FSYNC;

   logic       AXIS_TCLK;
   logic [31:0] AXIS_TDATA;
   logic       AXIS_TKEEP;
   logic       AXIS_TLAST;
    logic       AXIS_TREADY;
   logic [3:0]  AXIS_TSTRB;
   logic       AXIS_TVALID;
   logic [31:0] DEBUG;

    // Internal Signals
    logic clk;
    logic pixel_clk;
    logic rx_rst;
    localparam time clk_period = 4ns; // 200 MHz

    // File write signals
    logic o_valid;
    logic [7:0] o_add;
    logic [15:0] tst = 16'h0100;

    // Clock Generation
    always begin
        clk = 1;
        #(clk_period/2);
        clk = 0;
        #(clk_period/2);
    end

	always @(clk) begin
		sys_clk_p = clk;
		sys_clk_n = ~clk;
	end

    // Instantiate VHDL component (assuming mixed-language simulation support)
        fmc_mipi_top #(
        .N_MIPI_LANES(N_MIPI_LANES),
        .SERDES_DATA_WIDTH(SERDES_DATA_WIDTH),
        .ADD_DEBUG_OVERLAY(ADD_DEBUG_OVERLAY)
    ) inst_fmc_mipi_top (
        .sys_clk_p(sys_clk_p),
        .sys_clk_n(sys_clk_n),
        .sys_rst(sys_rst),
        .i2c3_cam_clk(i2c3_cam_clk),
        .monitor_1p2v(monitor_1p2v),
        .i2c2_cam_clk(i2c2_cam_clk),
        .i2c3_cam_dat(i2c3_cam_dat),
        .switch_data_lanes_vadj(switch_data_lanes_vadj),
        .switch_clock_lanes_vadj(switch_clock_lanes_vadj),
        .i2c_cam_dat(i2c_cam_dat),
        .i2c_cam_clk(i2c_cam_clk),
        .monitor_1p8v(monitor_1p8v),
        .cam0_rst(cam0_rst),
        .cam0_pwr(cam0_pwr),
        .cam0_mclk(cam0_mclk),
        .monitor_2p8v(monitor_2p8v),
        .monitor_3p3v(monitor_3p3v),
        .i2c2_cam_dat(i2c2_cam_dat),
        .en_mipi_out_data_vadj(en_mipi_out_data_vadj),
        .en_mipi_out_clock_vadj(en_mipi_out_clock_vadj),
        .enn_jt_gpio_input(enn_jt_gpio_input),
        .gpio17_out(gpio17_out),
        .gpio9_out(gpio9_out),
        .hs_c_d1_p(hs_c_d1_p),
        .hs_c_d1_n(hs_c_d1_n),
        .hs_c_d0_p(hs_c_d0_p),
        .hs_c_d0_n(hs_c_d0_n),
        .lp_c_d1_p(lp_c_d1_p),
        .lp_c_d1_n(lp_c_d1_n),
        .lp_c_d0_p(lp_c_d0_p),
        .lp_c_d0_n(lp_c_d0_n),
        .lp_c_clk_p(lp_c_clk_p),
        .lp_c_clk_n(lp_c_clk_n),
        .hs_d_d1_p(hs_d_d1_p),
        .hs_d_d1_n(hs_d_d1_n),
        .hs_d_d0_p(hs_d_d0_p),
        .hs_d_d0_n(hs_d_d0_n),
        .hs_d_clk_p(hs_d_clk_p),
        .hs_d_clk_n(hs_d_clk_n),
        .hs_c_clk_p(hs_c_clk_p),
        .hs_c_clk_n(hs_c_clk_n),
        .lp_d_d1_p(lp_d_d1_p),
        .lp_d_d1_n(lp_d_d1_n),
        .lp_d_d0_p(lp_d_d0_p),
        .lp_d_d0_n(lp_d_d0_n),
        .lp_d_clk_p(lp_d_clk_p),
        .lp_d_clk_n(lp_d_clk_n),
        .leds_debug(leds_debug),
        .dbg_io_1(dbg_io_1),
        .dbg_io_2(dbg_io_2),
        .dbg_io_3(dbg_io_3),
        .GPIO_SW_N(GPIO_SW_N),
        .GPIO_SW_E(GPIO_SW_E),
        .GPIO_SW_S(GPIO_SW_S),
        .GPIO_SW_W(GPIO_SW_W),
        .GPIO_SW_C(GPIO_SW_C)
    );

	aq_mipi_csi2rx_ultrascaleplus csirx (
        .RST_N(~sys_rst),

        .MIPI_CLK_P(hs_c_clk_p),
        .MIPI_CLK_N(hs_c_clk_n),

        .MIPI_LANE0_P(hs_c_d0_p),
        .MIPI_LANE0_N(hs_c_d0_n),
        .MIPI_LANE1_P(hs_c_d1_p),
        .MIPI_LANE1_N(hs_c_d1_n),

        .LP_LANE0_P(lp_c_d0_p),
        .LP_LANE0_N(lp_c_d0_n),
        .LP_LANE1_P(lp_c_d1_p),
        .LP_LANE1_N(lp_c_d1_n),

        .MIPI_CLK(MIPI_CLK),
        .PIXEL_NUM(PIXEL_NUM),
        .LINE_NUM(LINE_NUM),
        .DETECT(DETECT),
        .FS(FS),
        .FE(FE),
        .L0(L0),
        .L1(L1),
        .DET_D0(DET_D0),
        .DET_D1(DET_D1),

        .MIPI_LANE0(MIPI_LANE0),
        .MIPI_LANE1(MIPI_LANE1),

        .MIPI_FSYNC(MIPI_FSYNC),

        .AXIS_TCLK(AXIS_TCLK),
        .AXIS_TDATA(AXIS_TDATA),
        .AXIS_TKEEP(AXIS_TKEEP),
        .AXIS_TLAST(AXIS_TLAST),
        .AXIS_TREADY(AXIS_TREADY),
        .AXIS_TSTRB(AXIS_TSTRB),
        .AXIS_TVALID(AXIS_TVALID),

        .DEBUG(DEBUG)
    );

    // Test Stimulus
    initial begin
        #(clk_period*5);
		sys_rst = 1;
		rx_rst = 1;
		#(clk_period*5);
		#(clk_period*20);
		sys_rst = 0;
		#(clk_period*200);
		GPIO_SW_C = 1;
		#(clk_period*2000);
		#(clk_period*20000);
		#1s

		$finish;

    end

endmodule