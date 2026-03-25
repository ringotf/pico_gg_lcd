#pragma once


#define gg_clock_khz 32215.9 //32.215mhz game gear "sub-pixel" clock

#define pixels_in_scanline 250 //280 //300
#define scanlines_in_active_area  196 //180 //192

#define FRAME_SIZE_PIXELS (pixels_in_scanline * scanlines_in_active_area)
#define FRAME_SIZE_BYTES (FRAME_SIZE_PIXELS * 2)

#define v_lines_to_skip 51
#define h_pixels_to_skip 64

#define gg_pixel_width 160
#define gg_pixel_height 144
#define gg_pixel_x_offset 43 //28
#define gg_pixel_x_offset_dvi 43 //28


void fill_framebuffer_with_test_pattern();

void set_backlight(uint16_t level);
uint8_t config_backlight_pwm();