set_property IOSTANDARD LVDS [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_n]


set_property PACKAGE_PIN AB7 [get_ports sys_rst]
set_property PACKAGE_PIN AD12 [get_ports sys_clk_p]
set_property PACKAGE_PIN AD11 [get_ports sys_clk_n]
set_property IOSTANDARD LVCMOS15 [get_ports sys_rst]

#output delay constrains




#GPIO PUSHBUTTON SW ,if put the required LVCMOS15 error occures
set_property PACKAGE_PIN G12 [get_ports GPIO_SW_C]
set_property IOSTANDARD LVCMOS25 [get_ports GPIO_SW_C]
set_property PACKAGE_PIN AG5 [get_ports GPIO_SW_E]
set_property IOSTANDARD LVCMOS15 [get_ports GPIO_SW_E]
set_property PACKAGE_PIN AA12 [get_ports GPIO_SW_N]
set_property IOSTANDARD LVCMOS18 [get_ports GPIO_SW_N]
set_property PACKAGE_PIN AB12 [get_ports GPIO_SW_S]
set_property IOSTANDARD LVCMOS18 [get_ports GPIO_SW_S]
set_property PACKAGE_PIN AC6 [get_ports GPIO_SW_W]
set_property IOSTANDARD LVCMOS15 [get_ports GPIO_SW_W]



#if put the required LVCMOS15 error occures
set_property PACKAGE_PIN AB8 [get_ports {leds_debug[0]}]
set_property PACKAGE_PIN AA8 [get_ports {leds_debug[1]}]
set_property PACKAGE_PIN AC9 [get_ports {leds_debug[2]}]
set_property PACKAGE_PIN AB9 [get_ports {leds_debug[3]}]
set_property PACKAGE_PIN AE26 [get_ports {leds_debug[4]}]
set_property PACKAGE_PIN G19 [get_ports {leds_debug[5]}]
set_property PACKAGE_PIN E18 [get_ports {leds_debug[6]}]
set_property PACKAGE_PIN F16 [get_ports {leds_debug[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {leds_debug[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {leds_debug[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {leds_debug[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {leds_debug[0]}]
set_property IOSTANDARD LVCMOS25 [get_ports {leds_debug[7]}]
set_property IOSTANDARD LVCMOS25 [get_ports {leds_debug[6]}]
set_property IOSTANDARD LVCMOS25 [get_ports {leds_debug[5]}]
set_property IOSTANDARD LVCMOS25 [get_ports {leds_debug[4]}]


set_property PACKAGE_PIN G22 [get_ports cam0_mclk]
set_property IOSTANDARD LVCMOS25 [get_ports cam0_mclk]


set_property PACKAGE_PIN F22 [get_ports dbg_io_1]
set_property IOSTANDARD LVCMOS25 [get_ports dbg_io_1]

set_property PACKAGE_PIN H22 [get_ports dbg_io_2]
set_property IOSTANDARD LVCMOS25 [get_ports dbg_io_2]

set_property PACKAGE_PIN H21 [get_ports dbg_io_3]
set_property IOSTANDARD LVCMOS25 [get_ports dbg_io_3]

set_property PACKAGE_PIN C17 [get_ports cam0_pwr]
set_property IOSTANDARD LVCMOS25 [get_ports cam0_pwr]
set_property PACKAGE_PIN F17 [get_ports cam0_rst]
set_property IOSTANDARD LVCMOS25 [get_ports cam0_rst]
set_property PACKAGE_PIN A30 [get_ports en_mipi_out_clock_vadj]
set_property PACKAGE_PIN B30 [get_ports en_mipi_out_data_vadj]
set_property PACKAGE_PIN A25 [get_ports enn_jt_gpio_input]
set_property IOSTANDARD LVCMOS25 [get_ports en_mipi_out_clock_vadj]
set_property IOSTANDARD LVCMOS25 [get_ports en_mipi_out_data_vadj]
set_property IOSTANDARD LVCMOS25 [get_ports enn_jt_gpio_input]
set_property IOSTANDARD LVCMOS25 [get_ports gpio9_out]
set_property IOSTANDARD LVCMOS25 [get_ports gpio17_out]
set_property PACKAGE_PIN A18 [get_ports gpio9_out]
set_property PACKAGE_PIN B18 [get_ports gpio17_out]
set_property PACKAGE_PIN C25 [get_ports hs_c_clk_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_clk_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_clk_n]
set_property PACKAGE_PIN E28 [get_ports hs_c_d0_p]
set_property PACKAGE_PIN G28 [get_ports hs_c_d1_p]
set_property PACKAGE_PIN D21 [get_ports hs_d_clk_p]
set_property PACKAGE_PIN D22 [get_ports hs_d_d0_p]
set_property PACKAGE_PIN D16 [get_ports hs_d_d1_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_d0_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_d0_n]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_clk_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_clk_n]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_d1_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_d1_n]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_d1_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_c_d1_n]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_d0_p]
set_property IOSTANDARD BLVDS_25 [get_ports hs_d_d0_n]
set_property PACKAGE_PIN H26 [get_ports i2c2_cam_clk]
set_property PACKAGE_PIN H24 [get_ports i2c3_cam_clk]
set_property PACKAGE_PIN H27 [get_ports i2c3_cam_dat]
set_property PACKAGE_PIN G29 [get_ports i2c2_cam_dat]
set_property PACKAGE_PIN B20 [get_ports i2c_cam_clk]
set_property PACKAGE_PIN C20 [get_ports i2c_cam_dat]
set_property PACKAGE_PIN F18 [get_ports lp_c_clk_n]
set_property PACKAGE_PIN G18 [get_ports lp_c_clk_p]
set_property PACKAGE_PIN B24 [get_ports lp_c_d0_n]
set_property PACKAGE_PIN C24 [get_ports lp_c_d0_p]
set_property PACKAGE_PIN F27 [get_ports lp_c_d1_n]
set_property PACKAGE_PIN G27 [get_ports lp_c_d1_p]
set_property PACKAGE_PIN D19 [get_ports lp_d_clk_n]
set_property PACKAGE_PIN E19 [get_ports lp_d_clk_p]
set_property PACKAGE_PIN A27 [get_ports lp_d_d0_n]
set_property PACKAGE_PIN B27 [get_ports lp_d_d0_p]
set_property PACKAGE_PIN B29 [get_ports lp_d_d1_n]
set_property PACKAGE_PIN C29 [get_ports lp_d_d1_p]
set_property PACKAGE_PIN H25 [get_ports monitor_1p2v]
set_property PACKAGE_PIN G17 [get_ports monitor_1p8v]
set_property PACKAGE_PIN D26 [get_ports monitor_2p8v]
set_property PACKAGE_PIN C26 [get_ports monitor_3p3v]
set_property PACKAGE_PIN E30 [get_ports switch_clock_lanes_vadj]
set_property PACKAGE_PIN E29 [get_ports switch_data_lanes_vadj]
set_property IOSTANDARD LVCMOS25 [get_ports i2c2_cam_clk]
set_property IOSTANDARD LVCMOS25 [get_ports i2c2_cam_dat]
set_property IOSTANDARD LVCMOS25 [get_ports i2c3_cam_dat]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_clk_p]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_d0_p]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_d1_p]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_clk_p]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_d0_n]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_d0_p]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_d1_p]
set_property IOSTANDARD LVCMOS25 [get_ports monitor_1p8v]
set_property IOSTANDARD LVCMOS25 [get_ports monitor_3p3v]
set_property IOSTANDARD LVCMOS25 [get_ports switch_data_lanes_vadj]
set_property IOSTANDARD LVCMOS25 [get_ports i2c3_cam_clk]
set_property IOSTANDARD LVCMOS25 [get_ports i2c_cam_dat]
set_property IOSTANDARD LVCMOS25 [get_ports i2c_cam_clk]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_clk_n]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_d0_n]
set_property IOSTANDARD LVCMOS25 [get_ports lp_c_d1_n]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_clk_n]
set_property IOSTANDARD LVCMOS25 [get_ports monitor_2p8v]
set_property IOSTANDARD LVCMOS25 [get_ports switch_clock_lanes_vadj]
set_property IOSTANDARD LVCMOS25 [get_ports lp_d_d1_n]
set_property IOSTANDARD LVCMOS25 [get_ports monitor_1p2v]



