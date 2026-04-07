#pragma once


#define gg_clock_khz 32215.9 //32.215mhz game gear "sub-pixel" clock

#define pixels_in_scanline 256 
#define scanlines_in_active_area  193  

#define FRAME_SIZE_PIXELS (pixels_in_scanline * scanlines_in_active_area)
#define FRAME_SIZE_BYTES (FRAME_SIZE_PIXELS * 2)

#define gg_v_lines_to_skip 49
//#define gg_h_pixels_to_skip 64

#define gg_pixel_width 160
#define gg_pixel_height 144
#define gg_pixel_x_offset 40 
#define gg_pixel_x_offset_dvi 40 


#define sms_v_lines_to_skip 1
//#define sms_h_pixels_to_skip 90

#define sms_pixel_width 256
#define sms_pixel_height 192
#define sms_pixel_x_offset 0 
#define sms_pixel_x_offset_dvi 0 


#ifndef DVI_VERTICAL_REPEAT_GG
#define DVI_VERTICAL_REPEAT_GG 3
#endif

#ifndef DVI_VERTICAL_REPEAT_SMS
#define DVI_VERTICAL_REPEAT_SMS 2
#endif

extern bool is_gg;
extern bool gg_now;
extern bool last_gg;

void fill_framebuffer_with_test_pattern();

void set_backlight(uint16_t level);
uint8_t config_backlight_pwm();