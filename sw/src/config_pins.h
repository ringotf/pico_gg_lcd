//pin config in separate file for when differnt pin configs end up being needed for different pcb versions...

#pragma once

//libdvi output
#define dvi_d2_pins_base 0
#define dvi_d1_pins_base 2
#define dvi_d0_pins_base 4
#define dvi_clk_pins_base 6

//direct to lcd control
#define lcd_den 8
#define lcd_clk 9
#define lcd_rst 10

//gg video capture
#define gg_D1_pin 11 
#define gg_D2_pin 12 
#define gg_D3_pin 13 
#define gg_D4_pin 14 

#define gg_cl2_pin 15   //hsync
#define gg_dw_pin 16    //vsync
//#define gg_clk_pin 17   //sub-pixel RGB clock - not needed in current build, virtual clock is derived using PIO clock divider


//serial-to-parallel lcd shift registers
#define lcd_spi_latch 18
#define lcd_spi_clk 19
#define lcd_spi_mosi_1 23
#define lcd_spi_mosi_2 24
#define lcd_spi_mosi_3 25

//parallel-to-serial "SMS/GG" and button inputs shift register
#define btn_sr_miso 20
#define btn_sr_load 21
#define btn_sr_clk 22

//adc audio capture
#define gg_audio_r_pin 26
#define gg_audio_l_pin 27

//adc brightness controls
#define brightness_pot 28
#define lcd_dim 29

