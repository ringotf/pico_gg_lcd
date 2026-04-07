#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/structs/pwm.h"
#include "hardware/pwm.h"
//#include "hardware/interp.h"
#include "hardware/spi.h"

#include "../build/gg_capture.pio.h"
#include "../build/sms_capture.pio.h"
#include "../build/lcd_send_spi_2x.pio.h"
#include "../build/lcd_send_spi_4x.pio.h"
#include "font8x8_basic.h"

#include "libdvi/dvi.h"
#include "libdvi/dvi_serialiser.h"
#include "libdvi/dvi_timing.h"

#include "config_pins.h"
#include "main.h"


//Use two framebuffers to prevent tearing
uint16_t * framebuffer = (uint16_t *)(0x20000000 + (1024 * 40));
uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 40) + FRAME_SIZE_BYTES);

uint32_t dma_chan_fb1_write;
uint32_t dma_chan_fb1_reset;
uint32_t dma_chan_fb2_write;
uint32_t dma_chan_fb2_reset;

//
//some pio configs
//
#define btn_sr_spi spi0

#define lcd_send_pio pio0 
#define lcd_send_sm 2

#define lcd_send_clk_pio pio0
#define lcd_send_clk_sm 3

#define gg_capture_pio pio0
#define gg_capture_hblank_sm 0 
#define gg_capture_getdata_sm 1 


//some lcd configs for gg
#define lcd_pio_hscale_gg 4
#define lcd_hscale_factor_gg 1 / lcd_pio_hscale_gg

#define lcd_target_width_gg 640
#define lcd_send_width_gg 160 //lcd_target_width * lcd_hscale_factor

//add extra pixels to the h porch
#define lcd_hblank_sync_len_gg 2 * lcd_hscale_factor_gg // 2
#define lcd_hblank_front_len_gg 44 * lcd_hscale_factor_gg  //24
#define lcd_hblank_back_len_gg 42 * lcd_hscale_factor_gg  //12
#define lcd_hblank_len_gg lcd_hblank_front_len_gg + lcd_send_width_gg + lcd_hblank_back_len_gg //this is sync plus both porches plus active display area

#define lcd_active_lines_gg 160 // 160 * 3x scale = 480 lcd lines
#define lcd_vscale_factor_gg 3



//some lcd configs for sms
#define lcd_pio_hscale_sms 2
#define lcd_hscale_factor_sms 1 / lcd_pio_hscale_sms

#define lcd_target_width_sms 640
#define lcd_send_width_sms 320 //lcd_target_width * lcd_hscale_factor

//add extra pixels to the h porch
#define lcd_hblank_sync_len_sms 2 * lcd_hscale_factor_sms // 2
#define lcd_hblank_front_len_sms 44 * lcd_hscale_factor_sms  //24
#define lcd_hblank_back_len_sms 42 * lcd_hscale_factor_sms  //12
#define lcd_hblank_len_sms lcd_hblank_front_len_sms + lcd_send_width_sms + lcd_hblank_back_len_sms //this is sync plus both porches plus active display area

#define lcd_active_lines_sms 240 // 160 * 3x scale = 480 lcd lines
#define lcd_vscale_factor_sms 2



#define lcd_vblank_sync_lines 2 
#define lcd_vblank_front_lines 16 //22 
#define lcd_vblank_back_lines 14 //88 


//
//libdvi config
//

#define dvi_pio pio1
#define dvi_tmds_sm_0 0
#define dvi_tmds_sm_1 1
#define dvi_tmds_sm_2 2

// DVDD 1.2V (1.1V seems ok too)
//#define FRAME_WIDTH 320
//#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20

#define DVI_TIMING dvi_timing_640x480p_60hz

struct dvi_inst dvi0;

static struct dvi_serialiser_cfg pico_gg_lcd_conf = {
	.pio = dvi_pio,
	.sm_tmds = {dvi_tmds_sm_0, dvi_tmds_sm_1, dvi_tmds_sm_2},
	.pins_tmds = {dvi_d0_pins_base, dvi_d1_pins_base, dvi_d2_pins_base},
	.pins_clk = dvi_clk_pins_base,
	.invert_diffpairs = false
};

#define DVI_DMA_IRQ DMA_IRQ_0

#define DVI_AUDIO_CTS 28000
#define DVI_AUDIO_BUFFER_SIZE 256
audio_sample_t dvi_audio_buffer[DVI_AUDIO_BUFFER_SIZE];


//
//ADC audio capture
//
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_CHANNEL_COUNT 4 //2 audio + 1 brightness potentiometer, but needs to be power of 2 for dma and ring buffer to work ok 
#define AUDIO_BUFFER_BITS 11
#define AUDIO_BUFFER_SIZE (1 << AUDIO_BUFFER_BITS)
//#define AUDIO_BUFFER_SIZE 1024

__attribute__((aligned((1 << AUDIO_BUFFER_BITS))))
__attribute__((section(".time_critical.ram")))
static uint16_t audio_buffer[AUDIO_BUFFER_SIZE];

static const uint32_t audio_buffer_addr = (uint32_t)audio_buffer;


//brightness potentiometer value
volatile uint32_t brightness = 0;
uint8_t pwm_backlight_slice;
#define brightness_factor 3.3f  //brightness potentiometer only seems to go up to 1v

//
//lcd dimming using pwm to influence the 20v regulator
//
#define pwm_wrap_target			1024
#define pwm_default_level		800
#define pwm_min_level			0 //300
#define pwm_clk_div 			16


//
//button states
//
bool is_gg = true;
bool gg_now = true;
bool last_gg = true;

bool gg_start_now;
bool gg_btn1_now;
bool gg_btn2_now;
bool gg_btn_up_now;
bool gg_btn_dn_now;
bool gg_btn_lt_now;
bool gg_btn_rt_now;

bool gg_start_was;
bool gg_btn1_was;
bool gg_btn2_was;
bool gg_btn_up_was;
bool gg_btn_dn_was;
bool gg_btn_lt_was;
bool gg_btn_rt_was;

bool gg_start_changed;
bool gg_btn1_changed;
bool gg_btn2_changed;
bool gg_btn_up_changed;
bool gg_btn_dn_changed;
bool gg_btn_lt_changed;
bool gg_btn_rt_changed;

bool gg_start_pressed;
bool gg_btn1_pressed;
bool gg_btn2_pressed;
bool gg_btn_up_pressed;
bool gg_btn_dn_pressed;
bool gg_btn_lt_pressed;
bool gg_btn_rt_pressed;

bool gg_start_released;
bool gg_btn1_released;
bool gg_btn2_released;
bool gg_btn_up_released;
bool gg_btn_dn_released;
bool gg_btn_lt_released;
bool gg_btn_rt_released;

uint32_t startup_time;


//
//lcd update methods
//

static inline __attribute__ ((always_inline)) void send_lcd_pixel(uint16_t data)
{
	pio_sm_put(lcd_send_pio, lcd_send_sm, (uint32_t)data);
}

//used for blanking periods
static inline __attribute__ ((always_inline)) void send_4blank_lcd_pixel()
{
	while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);

}
static inline __attribute__ ((always_inline)) void send_2blank_lcd_pixel()
{
	while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);

}

volatile uint32_t for_nop_count = 0;

//changes the data-enable signal, but waits for any previous pixels to be completely sent through the shift registers via PIO interupt
static inline __attribute__ ((always_inline)) void lcd_den_set(bool lcd_den_value, bool skip_interrupt) 
{
	while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm));

	for(for_nop_count = 0; for_nop_count < 5; for_nop_count++)
	{
		__asm volatile("nop");
	}
	
	//wait for pio interrupt to flag it's stalled and waiting for bytes
	while(!skip_interrupt && !pio_interrupt_get(lcd_send_pio, 4));

	gpio_put(lcd_den, lcd_den_value);
	
}

/*
static inline __attribute__ ((always_inline)) uint32_t unpack(uint32_t rgb_value) {
	uint32_t temp = rgb_value >> 8;
	temp |= rgb_value << 20;
	temp |= rgb_value << 6;
	temp &= 0b111100000011110000001111;

	return temp;
}
*/


uint16_t * framebuffer_to_use;

//__attribute__ ((long_call, section (".time_critical"))) void update_lcd_gg(uint16_t * curr_framebuffer) {	
void update_lcd_gg(uint16_t * curr_framebuffer) {	

	//uint16_t spi_buffer[4] = {0x800, 0x800, 0x800, 0x800};  

	//Because of the LCD's orientation, we actually need to scan the image out upside down and flipped
	//base points to the bottom right of the original image
	uint32_t base = pixels_in_scanline + 48 - 52 + FRAME_SIZE_PIXELS - pixels_in_scanline * 51;

	static uint16_t line_diff = lcd_active_lines_gg - gg_pixel_height;
	uint16_t header_count = line_diff * 0.5;
	uint16_t footer_count = line_diff - header_count;

	uint32_t y_base;
	uint32_t y_base_offset = (gg_pixel_x_offset + (pixels_in_scanline - gg_pixel_width) * 0.5);
	uint32_t y_target;

	uint32_t blank_lines, blank_pixels, w, x, y;

	//might not need to skip interrupt?
	lcd_den_set(0, true);


	//start vsync pulse

	for(blank_lines = 0; blank_lines < lcd_vblank_sync_lines; blank_lines ++) {
		
		for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_gg; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}
	}
	//end vsync pulse

	//start vsync back porch
	for(blank_lines = 0; blank_lines < lcd_vblank_back_lines; blank_lines ++) {
		
		for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_gg; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

	}
	//end vsync back porch



	//footer goes at the top when the frame is upside-down

	//frame footer
	//do some blank footer lines....
	for (y = gg_pixel_height; y < gg_pixel_height + footer_count; y++)
	{
		for (w = 0; w < lcd_vscale_factor_gg; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_gg; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			//y_base = y * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			//y_base += y_base_offset;

			lcd_den_set(1, false);

			//blank line
			//for (uint32_t x = y_base + lcd_send_width; x > y_base; x -=4)
			for (x = 0 ; x < lcd_send_width_gg; x +=4)
			{
				send_4blank_lcd_pixel();
			}
			
			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_gg; blank_pixels ++) {				
				send_4blank_lcd_pixel();
			}
			
		}
	}


	//frame body
	//start frame data - this reads from bottom line to top line so the frame is sent upside-down	
	for (y = gg_pixel_height + gg_v_lines_to_skip; y > gg_v_lines_to_skip ; y--)	
	{
		for (w = 0; w < lcd_vscale_factor_gg; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_gg; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			//Where in the framebuffer the current scanline starts
			//uint32_t y_base = base - ((y * 310) >> 9) * pixels_in_scanline + 256;

			y_base = (y-1) * pixels_in_scanline;			
			y_base += y_base_offset;

			//Tell LCD we're starting the active portion of the scanline
			lcd_den_set(1, false);

			///unpack transforms 0b0000rrrrggggbbbb word into 0bbbb000000gggg000000rrrr word
			//Channel order is changed to bgr because that's what the LCD expects
			//Channel bits are spread out so we can do SIMD-within-a-word
			//The interpolator will do linear interpolation on all three channels at once, using a single 32-bit word
			//uint32_t framebuffer_pix_value = curr_framebuffer[y_base] & 4095;
			//uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);
			
			//for (uint32_t x = y_base; x < y_base + lcd_send_width; x +=4)
			for (x = y_base + lcd_send_width_gg-1; x >= y_base; x -=4)
			{

				//sub_pixel is a 24.8 bit fixed point coordinate
				
				//uint32_t sub_pixel = (x * 133);

				//Use integer part to pick a pixel from the framebuffer
				//uint32_t pixIndex = y_base - (sub_pixel >> 8);
				//uint32_t pixIndex = y_base - (x);
				//uint32_t pixIndex = y_base + (x);
				//uint32_t framebuffer_pix_value = curr_framebuffer[pixIndex] & 4095;

				//uint32_t new_pix_value = unpack(curr_framebuffer[pixIndex] & 4095);

				//Values to be interpolated
				//interp0->base[0] = old_pix_value;
				//interp0->base[1] = new_pix_value;

				//Fractional part of sub_pixel tells the interpolator how much of each pixel to use
				//interp0->accum[1] = sub_pixel & 0b11111100;

				//uint32_t pix_value = interp0->peek[1];
				//uint32_t pix_value = new_pix_value; //Uncomment this line to use nearest-neighbor instead of linear interpolation

				//old_pix_value = new_pix_value;
				
				//By the time we get here, the PIO is probably gonna be done sending the last pixel anyways
				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

				//This way, we only have to poll on the FIFO status once and then can just blast the three words at once
				//Seems to be significantly faster than pio_sm_put_blocking on each word
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);

				send_lcd_pixel(curr_framebuffer[x]);
				send_lcd_pixel(curr_framebuffer[x-1]);
				send_lcd_pixel(curr_framebuffer[x-2]);
				send_lcd_pixel(curr_framebuffer[x-3]);
					
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank front porch
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_gg; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}			
		}

	}

	//frame "header"
	//as the frame is output upside-down, the "header" goes at the bottom
	for (y = 0; y < header_count ; y++)
	{
		for (w = 0; w < lcd_vscale_factor_gg; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {	
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_gg; blank_pixels +=4) {		
				send_4blank_lcd_pixel();
			}
			
			//repeat the bottom lines at the top
			//y_base = (gg_pixel_height + y) * pixels_in_scanline;			
			//y_base += y_base_offset;
			//y_target = y_base + lcd_send_width;
			
			lcd_den_set(1, false);
			
			//for (uint32_t x = y_base; x < y_target; x +=4) {			
			//for (uint32_t x = y_target; x > y_base; x -=4) {	
			for (x = 0; x < lcd_send_width_gg; x +=4) {

				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;
				send_4blank_lcd_pixel();
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_gg; blank_pixels +=4) {				
				send_4blank_lcd_pixel();
			}

		}
	}


	//Start of vblank
	//lcd_den_set(1, false);

	//end frame data

	//start vsync front porch

	//Start of vblank 
	//lcd_den_set(0, false);

	//Send empty data for the entirety of vblank
	 
	for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_gg; blank_pixels +=4) {
		send_4blank_lcd_pixel();
	}

	for(blank_lines = 0; blank_lines < lcd_vblank_front_lines; blank_lines ++) {

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_gg; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}
	}
	//end vsync front porch
	
	//lcd_den_set(0, false);

}


/*
//The only thing that changes for SMS mode are the scaling constants
//However, putting those values in variables considerably slows down this function
//So, instead, we use magic constants baked into the code
__attribute__ ((long_call, section (".time_critical"))) void update_lcd_sms() {	
    while(!dma_channel_is_busy(dma_chan_fb1_write) && !dma_channel_is_busy(dma_chan_fb2_write)) ;
	uint16_t * curr_framebuffer;

	if(dma_channel_is_busy(dma_chan_fb1_write)) curr_framebuffer = framebuffer2;
	else curr_framebuffer = framebuffer;

	pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, scanlines_in_active_area + scanlines_to_skip - 1);
	pio_sm_exec(gg_capture_pio, gg_capture_hblank_sm, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_in_scanline * scanlines_to_skip;
	dma_hw->ch[3].transfer_count = pixels_in_scanline * scanlines_to_skip;

	uint32_t line_counter = 0;
	uint32_t base = pixels_in_scanline + 256 + 48 - 3 + (FRAME_SIZE_PIXELS - pixels_in_scanline * 2;

	for (uint32_t y = 0; y < 240; y ++)
	{
		uint32_t y_base = base - ((y * 413) >> 9) * pixels_in_scanline;

		gpio_put(lcd_den, 0);

		uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);

		for (uint32_t x = 0; x < 320; x ++)
		{
			uint32_t sub_pixel = (x * 210);
			uint32_t pixIndex = y_base - (sub_pixel >> 8);
			uint32_t new_pix_value = unpack(curr_framebuffer[pixIndex] & 4095);

			interp0->base[0] = old_pix_value;
			interp0->base[1] = new_pix_value;
			interp0->accum[1] = sub_pixel;

			uint32_t pix_value = interp0->peek[1];
			old_pix_value = new_pix_value;

			while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

			pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
		}

		gpio_put(lcd_den, 1);

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, 0);
		}
	}

	gpio_put(lcd_den, 1);

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_back_lines; wait_line ++) {
		for(uint32_t pixel = 0; pixel < lcd_send_width; pixel ++) {
			for(uint32_t channel = 0; channel < lcd_channels; channel ++) {
				pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, 0);
			}
		}

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, 0);
		}
	}
}
*/


//__attribute__ ((long_call, section (".time_critical"))) void update_lcd_sms(uint16_t * curr_framebuffer) {	
void update_lcd_sms(uint16_t * curr_framebuffer) {	

	//Because of the LCD's orientation, we actually need to scan the image out upside down and flipped
	//base points to the bottom right of the original image

	static uint16_t line_diff = lcd_active_lines_sms - sms_pixel_height;
	uint16_t header_count = line_diff * 0.5;
	uint16_t footer_count = line_diff - header_count;

	uint16_t border_width = (lcd_send_width_sms - sms_pixel_width) * 0.5;

	uint32_t y_base;
	uint32_t y_base_offset = (sms_pixel_x_offset + (pixels_in_scanline - sms_pixel_width) * 0.5);
	uint32_t y_target;

	uint32_t blank_lines, blank_pixels, w, x, y;

	//might not need to skip interrupt?
	lcd_den_set(0, true);

	//start vsync pulse

	for(blank_lines = 0; blank_lines < lcd_vblank_sync_lines; blank_lines ++) {
		
		for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_sms; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}
	}
	//end vsync pulse

	//start vsync back porch
	for(blank_lines = 0; blank_lines < lcd_vblank_back_lines; blank_lines ++) {
		
		for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_sms; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}

	}
	//end vsync back porch



	//footer goes at the top when the frame is upside-down

	//frame footer
	//do some blank footer lines....
	for (y = sms_pixel_height; y < sms_pixel_height + footer_count; y++)
	{
		for (w = 0; w < lcd_vscale_factor_sms; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_sms; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			//y_base = y * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			//y_base += y_base_offset;

			lcd_den_set(1, false);

			//blank line
			//for (uint32_t x = y_base + lcd_send_width; x > y_base; x -=4)
			for (x = 0 ; x < lcd_send_width_sms; x +=4)
			{
				send_4blank_lcd_pixel();
			}
			
			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_sms; blank_pixels ++) {				
				send_4blank_lcd_pixel();
			}
			
		}
	}


	//frame body
	//start frame data - this reads from bottom line to top line so the frame is sent upside-down	
	for (y = sms_pixel_height + sms_v_lines_to_skip; y > sms_v_lines_to_skip ; y--)	
	{
		for (w = 0; w < lcd_vscale_factor_sms; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_sms; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}

			//Where in the framebuffer the current scanline starts
			//uint32_t y_base = base - ((y * 310) >> 9) * pixels_in_scanline + 256;

			y_base = (y-1) * pixels_in_scanline;			
			y_base += y_base_offset;

			//Tell LCD we're starting the active portion of the scanline
			lcd_den_set(1, false);

			///unpack transforms 0b0000rrrrggggbbbb word into 0bbbb000000gggg000000rrrr word
			//Channel order is changed to bgr because that's what the LCD expects
			//Channel bits are spread out so we can do SIMD-within-a-word
			//The interpolator will do linear interpolation on all three channels at once, using a single 32-bit word
			//uint32_t framebuffer_pix_value = curr_framebuffer[y_base] & 4095;
			//uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);
			

			for(x = 0; x < border_width; x +=2 )
			{
				send_2blank_lcd_pixel();
			}

			//for (uint32_t x = y_base; x < y_base + lcd_send_width; x +=4)
			for (x = y_base + sms_pixel_width-1; x >= y_base; x -=2)
			{

				//By the time we get here, the PIO is probably gonna be done sending the last pixel anyways
				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

				//This way, we only have to poll on the FIFO status once and then can just blast the three words at once
				//Seems to be significantly faster than pio_sm_put_blocking on each word
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);

				send_lcd_pixel(curr_framebuffer[x]);
				send_lcd_pixel(curr_framebuffer[x-1]);
	
			}
			
			for(x = 0; x < border_width; x +=2 )
			{
				send_2blank_lcd_pixel();
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank front porch
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_sms; blank_pixels +=4) {
				send_4blank_lcd_pixel();
			}			
		}

	}

	//frame "header"
	//as the frame is output upside-down, the "header" goes at the bottom
	for (y = 0; y < header_count ; y++)
	{
		for (w = 0; w < lcd_vscale_factor_sms; w++)
		{
			for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {	
				send_4blank_lcd_pixel();
			}

			for(blank_pixels = 0; blank_pixels < lcd_hblank_back_len_sms; blank_pixels +=4) {		
				send_4blank_lcd_pixel();
			}
			
			//repeat the bottom lines at the top
			//y_base = (gg_pixel_height + y) * pixels_in_scanline;			
			//y_base += y_base_offset;
			//y_target = y_base + lcd_send_width;
			
			lcd_den_set(1, false);
			
			//for (uint32_t x = y_base; x < y_target; x +=4) {			
			//for (uint32_t x = y_target; x > y_base; x -=4) {	
			for (x = 0; x < lcd_send_width_sms; x +=4) {

				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;
				send_4blank_lcd_pixel();
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(blank_pixels = 0; blank_pixels < lcd_hblank_front_len_sms; blank_pixels +=4) {				
				send_4blank_lcd_pixel();
			}

		}
	}


	//Start of vblank
	//lcd_den_set(1, false);

	//end frame data

	//start vsync front porch

	//Start of vblank 
	//lcd_den_set(0, false);

	//Send empty data for the entirety of vblank
	 
	for(blank_pixels = 0; blank_pixels < lcd_hblank_sync_len_sms; blank_pixels +=4) {
		send_4blank_lcd_pixel();
	}

	for(blank_lines = 0; blank_lines < lcd_vblank_front_lines; blank_lines ++) {

		for(blank_pixels = 0; blank_pixels < lcd_hblank_len_sms; blank_pixels +=4) {
			send_4blank_lcd_pixel();
		}
	}
	//end vsync front porch
	
	//lcd_den_set(0, false);

}

void claim_pio_sm()
{
	//gg_capture_pio captures data from the GG's video bus
	pio_sm_claim(gg_capture_pio, gg_capture_hblank_sm);
	pio_sm_claim(gg_capture_pio, gg_capture_getdata_sm);


	//lcd_send_pio is used to send data to the LCD without hogging the CPU
	pio_sm_claim(lcd_send_pio, lcd_send_sm);

	//lcd_send_clk_pio clocks the lcd's clock...
	pio_sm_claim(lcd_send_clk_pio, lcd_send_clk_sm);
}


const pio_program_t * detect_hblank_ptr = 0;
uint8_t detect_hblank_offset;
pio_sm_config detect_hblank_config;

const pio_program_t * get_data_ptr = 0;
uint8_t get_data_offset;
pio_sm_config get_data_config;

void config_pios_capture()
{


	//
	//gg video capture pio programs
	//
	if(gg_now)
	{
		//separate pio programs for gg allows smaller frame buffer and less memory use overall
		detect_hblank_ptr = &detect_hblank_gg_program;
		detect_hblank_offset = pio_add_program(gg_capture_pio, detect_hblank_ptr);
		detect_hblank_config = detect_hblank_gg_program_get_default_config(detect_hblank_offset);
		
		get_data_ptr = &get_data_gg_program;
		get_data_offset = pio_add_program(gg_capture_pio, get_data_ptr);
		get_data_config = get_data_gg_program_get_default_config(get_data_offset);
	}
	else
	{
		//
		//sms video capture pio programs
		//
		detect_hblank_ptr = &detect_hblank_sms_program;
		detect_hblank_offset = pio_add_program(gg_capture_pio, detect_hblank_ptr);
		detect_hblank_config = detect_hblank_sms_program_get_default_config(detect_hblank_offset);
		
		get_data_ptr = &get_data_sms_program;
		get_data_offset = pio_add_program(gg_capture_pio, get_data_ptr);
		get_data_config = get_data_sms_program_get_default_config(get_data_offset);
	}

	sm_config_set_clkdiv(&detect_hblank_config, 1);

	//clock divider set to a multiple of the gg "sub-pixel" clock
	sm_config_set_clkdiv(&get_data_config, (DVI_TIMING.bit_clk_khz / gg_clock_khz) / 2);

	sm_config_set_in_pins(&get_data_config, gg_D1_pin);
	sm_config_set_in_shift(&get_data_config, false, true, 12);	//autopush after 12 bits have been read i.e. one RGB444 pixel	

	pio_sm_init(gg_capture_pio, gg_capture_hblank_sm, detect_hblank_offset, &detect_hblank_config);
	pio_sm_init(gg_capture_pio, gg_capture_getdata_sm, get_data_offset, &get_data_config);

	//push the number of lines per frame to capture
	pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, scanlines_in_active_area - 1);
	
	//encode a pull to save an instruction
	pio_sm_exec(gg_capture_pio, gg_capture_hblank_sm, pio_encode_pull(false, true));
	
	
	//push the number of pixels per line to capture
	pio_sm_put_blocking(gg_capture_pio, gg_capture_getdata_sm, pixels_in_scanline - 1); 
	
	//encode a pull to save an instruction
	pio_sm_exec(gg_capture_pio, gg_capture_getdata_sm, pio_encode_pull(false, true));

	//enable the state machines
	pio_enable_sm_mask_in_sync(gg_capture_pio, 0b1111);
}

const pio_program_t * lcd_send_spi_ptr = 0;
uint8_t lcd_send_spi_offset;
pio_sm_config lcd_send_spi_config;

const pio_program_t * lcd_send_clk_ptr = 0;
uint8_t lcd_send_clk_offset;
pio_sm_config lcd_send_clk_config;


void config_pios() 
{

	//stop and reset pios
	pio_set_sm_mask_enabled(gg_capture_pio, 0b1111, false);

	pio_clear_instruction_memory(gg_capture_pio); 
	//pio_clear_instruction_memory(pio1); //used by dvi output

	pio_interrupt_clear(gg_capture_pio, 0);
	pio_interrupt_clear(gg_capture_pio, 1);
	pio_interrupt_clear(gg_capture_pio, 2);
	pio_interrupt_clear(gg_capture_pio, 3);
	pio_interrupt_clear(gg_capture_pio, 4);
	pio_interrupt_clear(gg_capture_pio, 5);
	pio_interrupt_clear(gg_capture_pio, 6);	
	pio_interrupt_clear(gg_capture_pio, 7);

	pio_sm_clear_fifos(lcd_send_pio, lcd_send_sm);
	pio_sm_clear_fifos(lcd_send_pio, lcd_send_clk_sm);

	pio_sm_clear_fifos(gg_capture_pio, gg_capture_hblank_sm);
	pio_sm_clear_fifos(gg_capture_pio, gg_capture_getdata_sm);

	pio_remove_program(lcd_send_pio, lcd_send_spi_ptr, lcd_send_spi_offset);
	pio_remove_program(lcd_send_pio, lcd_send_clk_ptr, lcd_send_clk_offset);
	
	pio_remove_program(gg_capture_pio, detect_hblank_ptr, detect_hblank_offset);
	pio_remove_program(gg_capture_pio, get_data_ptr, get_data_offset);

	sleep_ms(100);

	if(gg_now)
	{
		//4x video output horizontal scaling
		lcd_send_spi_ptr = &lcd_send_spi_4x_program;
		lcd_send_clk_ptr = &lcd_send_clk_4x_program;

		lcd_send_spi_offset = pio_add_program(lcd_send_pio, lcd_send_spi_ptr);
		lcd_send_spi_config = lcd_send_spi_4x_program_get_default_config(lcd_send_spi_offset);

		lcd_send_clk_offset = pio_add_program(lcd_send_pio, lcd_send_clk_ptr);
		lcd_send_clk_config = lcd_send_clk_4x_program_get_default_config(lcd_send_clk_offset);
	}
	else
	{
		//2x video output horizontal scaling
		lcd_send_spi_ptr = &lcd_send_spi_2x_program;
		lcd_send_clk_ptr = &lcd_send_clk_2x_program;

		lcd_send_spi_offset = pio_add_program(lcd_send_pio, lcd_send_spi_ptr);
		lcd_send_spi_config = lcd_send_spi_2x_program_get_default_config(lcd_send_spi_offset);

		lcd_send_clk_offset = pio_add_program(lcd_send_pio, lcd_send_clk_ptr);
		lcd_send_clk_config = lcd_send_clk_2x_program_get_default_config(lcd_send_clk_offset);
	}


	sm_config_set_clkdiv(&lcd_send_spi_config, 1);
	sm_config_set_out_pins(&lcd_send_spi_config, lcd_spi_mosi_1, 3); //shift register's master-out-slave-in pins
	sm_config_set_sideset_pins(&lcd_send_spi_config, lcd_spi_latch);

	sm_config_set_out_shift(&lcd_send_spi_config, true, true, 12); //auto-pull after 12bits have been read i.e. one RGB444 pixel

	sm_config_set_clkdiv(&lcd_send_clk_config, 1);
	sm_config_set_set_pins(&lcd_send_clk_config, lcd_clk, 1);


	//pins for sending to the shift registers
	pio_gpio_init(lcd_send_pio, lcd_spi_latch);
	pio_gpio_init(lcd_send_pio, lcd_spi_clk);
	pio_gpio_init(lcd_send_pio, lcd_spi_mosi_1);
	pio_gpio_init(lcd_send_pio, lcd_spi_mosi_2);
	pio_gpio_init(lcd_send_pio, lcd_spi_mosi_3);
		
	pio_gpio_init(lcd_send_clk_pio, lcd_clk);

	pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_spi_latch), (1 << lcd_spi_latch) );
	pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_spi_mosi_1), (1 << lcd_spi_mosi_1) );
	pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_spi_mosi_2), (1 << lcd_spi_mosi_2) );
	pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_spi_mosi_3), (1 << lcd_spi_mosi_3) );
	pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_spi_clk), (1 << lcd_spi_clk) );

	pio_sm_set_pindirs_with_mask(lcd_send_clk_pio, lcd_send_clk_sm, (1 << lcd_clk), (1 << lcd_clk) );

	pio_sm_init(lcd_send_pio, lcd_send_sm, lcd_send_spi_offset, &lcd_send_spi_config);
	pio_sm_set_enabled(lcd_send_pio, lcd_send_sm, true);

	pio_sm_init(lcd_send_clk_pio, lcd_send_clk_sm, lcd_send_clk_offset, &lcd_send_clk_config);
	pio_sm_set_enabled(lcd_send_clk_pio, lcd_send_clk_sm, true);

	

	config_pios_capture();
	
}

void claim_dma()
{
	for(uint32_t c = 0; c < 12; c++) {
		dma_channel_cleanup(c);
    	dma_channel_unclaim(c);
	}

	//DVI code uses claim_unused_channel, so we must as well, instead of explicitly picking DMA channels
	//DVI audio also claims 2 dma channels
	dma_chan_fb1_write = dma_claim_unused_channel(true);
	dma_chan_fb1_reset = dma_claim_unused_channel(true);
	dma_chan_fb2_write = dma_claim_unused_channel(true);
	dma_chan_fb2_reset = dma_claim_unused_channel(true);
}

void config_dma() {

	dma_channel_cleanup(dma_chan_fb1_write);
	dma_channel_cleanup(dma_chan_fb1_reset);
	dma_channel_cleanup(dma_chan_fb2_write);
	dma_channel_cleanup(dma_chan_fb2_reset);

	//PIO0 SM2 is the one that actually sends out pixel data

	//Now we can save the scanlines that actually have image data into the framebuffer
	dma_channel_config f = dma_channel_get_default_config(dma_chan_fb1_write);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan_fb1_reset);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(gg_capture_pio, gg_capture_getdata_sm, false));
	dma_channel_configure(
			dma_chan_fb1_write,
			&f,
			//Note that we're filling the first framebuffer here
			//There's two of them, for double buffering, so we don't get screen tearing
			&framebuffer[0],
			&gg_capture_pio->rxf[gg_capture_getdata_sm],
			//Again, the active area has border scanlines that don't actually contain game graphics
			//We only save the *real* scanlines into the framebuffer
			//Then chain to dma_chan_fb1_reset
			FRAME_SIZE_PIXELS,
			false);

	//Reset dma_chan_fb1_write's write address register to point to the start of framebuffer0
	//Then chain to dma_chan3
	dma_channel_config e = dma_channel_get_default_config(dma_chan_fb1_reset);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	channel_config_set_chain_to(&e, dma_chan_fb2_write);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan_fb1_reset,
			&e,
			&(dma_hw->ch[dma_chan_fb1_write].write_addr),
			&framebuffer,
			1,
			false);


	//Save active scanlines into second framebuffer
	f = dma_channel_get_default_config(dma_chan_fb2_write);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan_fb2_reset);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(gg_capture_pio, gg_capture_getdata_sm, false));
	dma_channel_configure(
			dma_chan_fb2_write,
			&f,
			&framebuffer2[0],
			&gg_capture_pio->rxf[gg_capture_getdata_sm],
			FRAME_SIZE_PIXELS,
			false);

	//Reset dma_chan_fb2_write's write address and chain into dma_chan_fb1_write to restart the whole DMA chain
	e = dma_channel_get_default_config(dma_chan_fb2_reset);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	channel_config_set_chain_to(&e, dma_chan_fb1_write);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan_fb2_reset,
			&e,
			&(dma_hw->ch[dma_chan_fb2_write].write_addr),
			&framebuffer2,
			1,
			false);
			


}


struct repeating_timer adc_timer;
struct repeating_timer dvi_audio_timer;
volatile uint16_t audio_write_pos = 0;
volatile uint16_t audio_read_pos = 0;

volatile int64_t accum_l = 0;
volatile int64_t accum_r = 0;
volatile uint16_t decimate = 0;
static int16_t prev_l = 0;

static const uint16_t audio_bias_midpoint = 2048; //ADC is 0-4095 2048 is 1.65v midpoint of 3.3v
//uint16_t audio_bias_midpoint = 1900; //1.53v
//static const int16_t audio_bias_midpoint = 1948; //1.57v
//uint16_t audio_bias_midpoint = 1775; //1.43v
//uint16_t audio_bias_midpoint = 1737; //1.40v
//uint16_t audio_bias_midpoint = 1514; //1.22v
//uint16_t audio_bias_midpoint = 1365; //1365 is midpoint of 1.1v
//uint16_t audio_bias_midpoint = 1340; //1.08v
//uint16_t audio_bias_midpoint = 1290; //1.04v
//uint16_t audio_bias_midpoint = 1240; //1.0v


/*
//some audio filter stuff but idk might be worth using

#define DC_FILTER_ALPHA     0.0005f       // very slow — time constant ~ 45 seconds at 44 kHz
                                          // smaller = slower / lower cutoff (0.0001–0.001 range)
#define DC_FILTER_ALPHA_INT (int32_t)(DC_FILTER_ALPHA * 32768.0f)   // Q15 fixed-point

// State
static int32_t dc_lowpass_l = 2048 << 15;   // initial mid-scale, scaled ×32768
static int32_t dc_lowpass_r = 2048 << 15;   // initial mid-scale, scaled ×32768
static int32_t lowpass_error = 0;
static int32_t lowpass_center = 0;

// Process one sample (call this for every ADC reading)
int16_t __not_in_flash_func(process_slow_iir_dc_block_l)(int32_t raw_adc) {
    // Update low-pass estimate (very slow)
    lowpass_error = (raw_adc << 15) - dc_lowpass_l;                    // scaled error
    dc_lowpass_l += (DC_FILTER_ALPHA_INT * lowpass_error) >> 15;               // accumulate

    // Center the sample
    lowpass_center = raw_adc - (dc_lowpass_l >> 15);

    // Optional clip
    if (lowpass_center > 2047) lowpass_center = 2047;
    if (lowpass_center < -2048) lowpass_center = -2048;

    return (int16_t)lowpass_center;
}

int16_t __not_in_flash_func(process_slow_iir_dc_block_r)(int32_t raw_adc) {
    // Update low-pass estimate (very slow)
    lowpass_error = (raw_adc << 15) - dc_lowpass_r;                    // scaled error
    dc_lowpass_r += (DC_FILTER_ALPHA_INT * lowpass_error) >> 15;               // accumulate

    // Center the sample
    lowpass_center = raw_adc - (dc_lowpass_r >> 15);

    // Optional clip
    if (lowpass_center > 2047) lowpass_center = 2047;
    if (lowpass_center < -2048) lowpass_center = -2048;

    return (int16_t)lowpass_center;
}
*/


int adc_dma_chan_sample = -1;
int adc_dma_chan_control = -1;

bool __not_in_flash_func(dvi_audio_timer_callback_dma)(struct repeating_timer *t)
{

	//while(true)
	{
		//get how many audio samples for the dvi buffer
		int size = get_write_size(&dvi0.audio_ring, true);
		if(size == 0) return true;
		if(size >= DVI_AUDIO_BUFFER_SIZE)
		{
			size = DVI_AUDIO_BUFFER_SIZE;
		}

		//get where we need to write the audio sample to
		audio_sample_t *audio_ptr = get_write_pointer(&dvi0.audio_ring);
		uint32_t audio_offset = get_write_offset(&dvi0.audio_ring);
		audio_sample_t sample;

		//Should be using trans_count as write_addr is unreliable
		uint32_t current_trans_count = dma_hw->ch[adc_dma_chan_sample].transfer_count;
		uint32_t samples_written = AUDIO_BUFFER_SIZE - current_trans_count;

		audio_write_pos = samples_written - (samples_written % 2);
		
		//printf("Trans Count: %u | Buffer Size: %u | samples requested: %u | Samples Available: %d | write_pos: %u | read_pos: %u \n",
         //  current_trans_count, AUDIO_BUFFER_SIZE, size, audio_write_pos-audio_read_pos, audio_write_pos, audio_read_pos);
		//printf("Trans Count: %u | Buffer Size: %u | samples requested: %u | Samples written: %u | write_pos: %u | read_pos: %u | FIFO level: %d\n",
        //   current_trans_count, AUDIO_BUFFER_SIZE, size, samples_written, audio_write_pos, audio_read_pos, adc_fifo_get_level());
		//printf("DMA addr: 0x%08X | Samples written: %u | write_pos: %u | read_pos: %u | FIFO level: %d | Request Size: %d | Buffer Size: %d\n",
        //   current_dma_addr, samples_written, audio_write_pos, audio_read_pos, adc_fifo_get_level(), size, AUDIO_BUFFER_SIZE);
		
		for(int cnt = 0; cnt < size; cnt++)
		{
			if(audio_write_pos == audio_read_pos)
			{
				//break;
				 sample.channels[0] = 0; //silence on underrun?
				 sample.channels[1] = 0;
			}
			else 
			{				
				//left audio channel
				sample.channels[0] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ;//left
				//sample.channels[1] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ; //right - duplicate left for test
				//sample.channels[0] = process_slow_iir_dc_block_l( (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint );

				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//right audio channel
				sample.channels[1] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ; //right
				//sample.channels[0] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ;//left - duplicate right for test
				//sample.channels[1] = process_slow_iir_dc_block_r( (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint );

				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//update the brightness potentiometer value
				brightness = (int16_t)(audio_buffer[audio_read_pos]);
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//4th sample just ignore - needed for power of 2 adc dma round robin buffer size			
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

		
				audio_offset = (audio_offset + 1) % (DVI_AUDIO_BUFFER_SIZE - 1);
			}

			*audio_ptr++ = sample;
			increase_write_pointer(&dvi0.audio_ring, 1);
			audio_ptr = get_write_pointer(&dvi0.audio_ring);

		}

	}
	
    return true;
}


void config_audio()
{

	for(uint32_t i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audio_buffer[i] = 2048;
	}

	adc_init();
	adc_gpio_init(gg_audio_l_pin); //enable adc and disabled gpio on these pins
	adc_gpio_init(gg_audio_r_pin);
	
	adc_set_temp_sensor_enabled(false);

	adc_set_round_robin(0b1111); //sample adc pins 1 and 2 i.e. 26/27 and 28 for brightness potentiometer, 29 is ignored but needs to be read to fit everything in a power of 2 buffer

	float clk_div = (48 * 1000 * 1000) / (AUDIO_SAMPLE_RATE * (AUDIO_CHANNEL_COUNT)); //need to include the brightness potentiometer
	adc_set_clkdiv(clk_div - 1.0f);
		
	adc_fifo_setup(true, true, 1, false, false);


	//
	//adc dma config
	//
	adc_dma_chan_sample = dma_claim_unused_channel(true);	
	adc_dma_chan_control = dma_claim_unused_channel(true);

	dma_channel_config adc_dma_config_sample = dma_channel_get_default_config(adc_dma_chan_sample);
	channel_config_set_transfer_data_size(&adc_dma_config_sample, DMA_SIZE_16);
	channel_config_set_read_increment(&adc_dma_config_sample, false);
	channel_config_set_write_increment(&adc_dma_config_sample, true);
	channel_config_set_dreq(&adc_dma_config_sample, DREQ_ADC);
	channel_config_set_ring(&adc_dma_config_sample, true, AUDIO_BUFFER_BITS + 1);
	channel_config_set_chain_to(&adc_dma_config_sample, adc_dma_chan_control);
	channel_config_set_enable(&adc_dma_config_sample, true);

	dma_channel_configure(adc_dma_chan_sample, 
		&adc_dma_config_sample, 
		audio_buffer, 
		&adc_hw->fifo, 
		AUDIO_BUFFER_SIZE, 
		false);


	dma_channel_config adc_dma_config_control = dma_channel_get_default_config(adc_dma_chan_control);
	channel_config_set_transfer_data_size(&adc_dma_config_control, DMA_SIZE_32);
	channel_config_set_read_increment(&adc_dma_config_control, false);
	channel_config_set_write_increment(&adc_dma_config_control, false);
	channel_config_set_dreq(&adc_dma_config_control, DREQ_FORCE);
	channel_config_set_chain_to(&adc_dma_config_control, adc_dma_chan_sample);
	channel_config_set_enable(&adc_dma_config_control, true);

	dma_channel_configure(adc_dma_chan_control, 
		&adc_dma_config_control, 
		&dma_hw->ch[adc_dma_chan_sample].al2_write_addr_trig, 
		&audio_buffer_addr, 
		1, 
		false);


	dma_channel_start(adc_dma_chan_control);

	adc_run(true);
	
	//timer for sending dvi audio
	add_repeating_timer_ms(1, dvi_audio_timer_callback_dma, NULL, &dvi_audio_timer);

}


//
//backlight dimming pwm config
//
uint16_t backlight_level = pwm_default_level;

void set_backlight(uint16_t level)
{
	//printf("backlight set to %i\n", level);

	level = (level > pwm_wrap_target) ? pwm_wrap_target : (level < pwm_min_level ) ? pwm_min_level : level;
	backlight_level = level;
	
	pwm_set_chan_level(pwm_backlight_slice, pwm_gpio_to_channel(lcd_dim), backlight_level);
}

uint8_t config_backlight_pwm() 
{
	gpio_set_function(lcd_dim, GPIO_FUNC_PWM);
	pwm_backlight_slice = pwm_gpio_to_slice_num(lcd_dim);

	pwm_config config = pwm_get_default_config();
	pwm_config_set_phase_correct(&config, false);	//default is false anyway
	pwm_config_set_clkdiv_int(&config, pwm_clk_div);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	
	pwm_config_set_wrap(&config, pwm_wrap_target);
	
	pwm_init(pwm_backlight_slice, &config, true);
	
	set_backlight(backlight_level);

	return pwm_backlight_slice;
}

	

// void config_interp() {
// 	//Claim both lanes
// 	interp_claim_lane_mask(interp0, 0b11);

// 	interp_config cfg = interp_default_config();
// 	//Only interpolator 0 has " blend " mode, which does linear interpolation
// 	interp_config_set_blend(&cfg, true);
// 	interp_set_config(interp0, 0, &cfg);

// 	cfg = interp_default_config();
// 	interp_set_config(interp0, 1, &cfg);
// }

void init_lcd() {
	sleep_ms(100);

	//gpio_put(lcd_rst, 1);
	sleep_ms(10);
	gpio_put(lcd_rst, 0);
	sleep_ms(10);
	gpio_put(lcd_rst, 1);
	sleep_ms(100);
}


//
//some simple but slow drawing methods
//

char* title = "PICO GG LCD";
const uint8_t font_size = 8;
const uint16_t font_color = 0xDDD;
const uint16_t overlay_color_base = 0x066F;
const uint8_t overlay_item_count = 4;
const uint8_t overlay_height = (font_size + 4) * overlay_item_count ;
const uint16_t overlay_ypos = gg_pixel_height + gg_v_lines_to_skip - overlay_height;
const uint16_t overlay_xpos = gg_pixel_x_offset + (pixels_in_scanline - gg_pixel_width) * 0.5;

//simple and slow character drawing
void inline __attribute__ ((always_inline)) draw_char(uint16_t * current_framebuffer, uint16_t x, uint16_t y, const char c, uint16_t color)
{
	if(c < 0x20 || c > 127) return;

	for(uint16_t row = 0; row < font_size; row++)
	{
		char line = font8x8_basic[c - 0x20][row];
		uint32_t y_base = ((y + row) * pixels_in_scanline) + x;

		for(uint16_t col = 0; col < font_size; col++)
		{
			if(line & (1 << col))
			{
				uint32_t pos = y_base + col;
				current_framebuffer[pos] = color;
			}
		}

	}
}

void draw_string(uint16_t * current_framebuffer, uint16_t x, uint16_t y, const char *str, uint16_t color)
{
	while(*str)
	{
		draw_char(current_framebuffer, x, y, *str++, color);
		x+=font_size;
	}
}

void draw_rectangle_empty(uint16_t * current_framebuffer, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint32_t ypos = (y * pixels_in_scanline) + x;
	uint32_t ypos2 = ((y + h - 2) * pixels_in_scanline) + x;
	uint32_t xpos_base = (y * pixels_in_scanline) + x;
	uint32_t xpos2_base = (y * pixels_in_scanline) + x + w - 1;
	
	for(uint32_t line = 0; line < w-1;line++)
	{
		//top line
		current_framebuffer[ypos + line] = color;
		
		//bottom line
		current_framebuffer[ypos2 + line] = color;
	}

	for(uint32_t line = 0; line < h-1; line++)
	{
		//left line
		current_framebuffer[xpos_base + (line * pixels_in_scanline)] = color;

		//right line 		
		current_framebuffer[xpos2_base + (line * pixels_in_scanline)] = color;
	}

}


int8_t menuItemIndex = 0;
int8_t menuItemCount = 3;

//
//draw a basic menu that doesnt do anything but exit
//
void draw_main_menu(uint16_t * current_framebuffer, uint8_t ypos)
{
	if(gg_btn_up_pressed)
	{
		menuItemIndex++;
	}
	if(gg_btn_dn_pressed)
	{
		menuItemIndex--;
	}
	
	menuItemIndex = MAX(0, MIN(menuItemCount-1, menuItemIndex));
	
	//draw some menu items
	if(menuItemIndex==2)		
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4, "1. Some Option *", font_color);
	}
	else
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4, "1. Some Option", font_color);
	}
	

	if(menuItemIndex == 1)
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4 + font_size + 4, "2. Another Option *", font_color);
	}
	else
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4 + font_size + 4, "2. Another Option", font_color);
	}

	if(menuItemIndex == 0)
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4 + font_size + 4 + font_size + 4, "Exit! *", font_color);
	}
	else
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4 + font_size + 4 + font_size + 4, "Exit?", font_color);
	}

}

char fps_str[8];
void draw_overlay(uint16_t * current_framebuffer, uint8_t fps)
{
	//draw a background
	for(uint32_t y = overlay_ypos; y < overlay_ypos + overlay_height; y++) 
	{
		//uint16_t row_val = 15;

		for(uint32_t x = 0; x < pixels_in_scanline; x++) {
			current_framebuffer[x + (y * pixels_in_scanline)]  = overlay_color_base;
			current_framebuffer[x + (y * pixels_in_scanline)]  = overlay_color_base;
		}
	}	

	//draw some text
	draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + 2, title, font_color);
	sprintf(fps_str, "%i fps", fps);

	draw_string(current_framebuffer, overlay_xpos + font_size*2 + strlen(title) * 8, overlay_ypos + 2, fps_str, font_color);

	draw_main_menu(current_framebuffer, overlay_ypos + font_size + 4);

}


//
//draws a pattern of colour bars vertically with varying intensity horizontally
//fills the frame buffer which is larger than the lcd, appears clipped in the output is normal
//
void fill_framebuffer_with_test_pattern() {
	uint16_t test_divs = pixels_in_scanline / 8;	
	uint16_t row_size = scanlines_in_active_area / 5;
	uint16_t row_diff = 3;
	for(uint32_t y = 0; y < scanlines_in_active_area; y++) {
		uint16_t row_val = (y / row_size);
		row_val = row_diff * row_val;
		row_val = 15 - row_val;

		//uint16_t row_val = 15;

		for(uint32_t x = 0; x < pixels_in_scanline; x++) {
			uint16_t pixel = 0;

			if(x < test_divs)
			{
				//white
				pixel |= row_val;
				pixel |= (row_val << 8);
				pixel |= (row_val << 4);
			}
			else if (x < test_divs * 2)
			{
				//yellow
				pixel |= (row_val << 8);
				pixel |= (row_val << 4);
				
			}
			else if (x < test_divs * 3)
			{
				//teal
				pixel |= row_val;
				pixel |= (row_val << 4);
			
			}
			else if (x < test_divs * 4)
			{
				//green
				pixel |= (row_val << 4);
			}
			else if (x < test_divs * 5)
			{
				//purple
				pixel |= row_val;
				pixel |= (row_val << 8);
			}
			else if (x < test_divs * 6)
			{
				//red
				pixel |= (row_val << 8);
			}
			else if (x < test_divs * 7)
			{
				//blue
				pixel |= row_val;
			}
			

			framebuffer[x + (y * pixels_in_scanline)]  = pixel;
			//framebuffer2[x + (y * pixels_in_scanline)]  = ~pixel;
			framebuffer2[x + (y * pixels_in_scanline)]  = pixel;

		}
	}

	//draws an outline around the rendered portion, in theory
	if(gg_now)
	{
		draw_rectangle_empty(framebuffer, gg_pixel_x_offset + 45, gg_v_lines_to_skip, gg_pixel_width, gg_pixel_height, 0xFFF);
		draw_rectangle_empty(framebuffer2, gg_pixel_x_offset + 45, gg_v_lines_to_skip, gg_pixel_width, gg_pixel_height, 0xFFF);
	}
	else
	{
		draw_rectangle_empty(framebuffer, sms_pixel_x_offset, sms_v_lines_to_skip, sms_pixel_width, sms_pixel_height, 0xFFF);
		draw_rectangle_empty(framebuffer2, sms_pixel_x_offset, sms_v_lines_to_skip, sms_pixel_width, sms_pixel_height, 0xFFF);
	}

}



void read_in_spi()
{
	
	gpio_put(btn_sr_clk, 0);

	gpio_put(btn_sr_load, 0);
	__asm("nop"); __asm("nop");
	//sleep_ms(1);

	gpio_put(btn_sr_load, 1);
	__asm("nop"); __asm("nop");

	//sleep_ms(1);
	
	gpio_put(btn_sr_clk, 1);
	__asm("nop"); __asm("nop");
	
	//sleep_ms(1);

	gpio_put(btn_sr_clk, 0);
	__asm("nop"); __asm("nop");

	//sleep_ms(1);

	uint8_t data;
	spi_read_blocking(btn_sr_spi, 0xFF , &data, 1);

	//printf("SPI IN DATA %i\n", data);

	gg_btn_dn_was = gg_btn_dn_now;
	gg_start_was = gg_start_now;
	gg_btn2_was = gg_btn2_now;
	gg_btn1_was = gg_btn1_now;
	gg_btn_lt_was = gg_btn_lt_now;
	gg_btn_rt_was = gg_btn_rt_now;
	gg_btn_up_was = gg_btn_up_now;

	gg_btn_dn_now = !(data & 0x1);
	gg_start_now = !(data & 0x2);
	gg_btn2_now = !(data & 0x4);
	gg_btn1_now = !(data & 0x8);
	gg_btn_lt_now = !(data & 0x10);
	gg_btn_rt_now = !(data & 0x20);
	gg_btn_up_now = !(data & 0x40);
	
	gg_btn_dn_changed = gg_btn_dn_now != gg_btn_dn_was;
	gg_start_changed = gg_start_now != gg_start_was;
	gg_btn2_changed = gg_btn2_now != gg_btn2_was;
	gg_btn1_changed = gg_btn1_now != gg_btn1_was;
	gg_btn_lt_changed = gg_btn_lt_now != gg_btn_lt_was;
	gg_btn_rt_changed = gg_btn_rt_now != gg_btn_rt_was;
	gg_btn_up_changed = gg_btn_up_now != gg_btn_up_was;

	gg_btn_dn_pressed = gg_btn_dn_changed && gg_btn_dn_now;
	gg_btn_dn_released = gg_btn_dn_changed && !gg_btn_dn_now;

	gg_start_pressed = gg_start_changed && gg_start_now;
	gg_start_released = gg_start_changed && !gg_start_now;

	gg_btn2_pressed = gg_btn2_changed && gg_btn2_now;
	gg_btn2_released = gg_btn2_changed && !gg_btn2_now;

	gg_btn1_pressed = gg_btn1_changed && gg_btn1_now;
	gg_btn1_released = gg_btn1_changed && !gg_btn1_now;

	gg_btn_lt_pressed = gg_btn_lt_changed && gg_btn_lt_now;
	gg_btn_lt_released = gg_btn_lt_changed && !gg_btn_lt_now;

	gg_btn_rt_pressed = gg_btn_rt_changed && gg_btn_rt_now;
	gg_btn_rt_released = gg_btn_rt_changed && !gg_btn_rt_now;

	gg_btn_up_pressed = gg_btn_up_changed && gg_btn_up_now;
	gg_btn_up_released = gg_btn_up_changed && !gg_btn_up_now;

	
	gg_now = !(data & 0x80); //idk somehow this is wrong way around, it should need negating


}


void config_in_spi()
{
	gpio_init(btn_sr_miso);
	gpio_init(btn_sr_load);
	gpio_init(btn_sr_clk);

	gpio_set_dir(btn_sr_load, GPIO_OUT);

	spi_init(btn_sr_spi, 10*1000*1000);
	//spi_init(in_sr_spi, 10);
	gpio_set_function(btn_sr_miso, GPIO_FUNC_SPI);
	gpio_set_function(btn_sr_clk, GPIO_FUNC_SPI);

	spi_set_format(btn_sr_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}


uint32_t last_frame_time = 0;
uint32_t last_frame_start = 0;

//
//primary lcd update loop
//
void __not_in_flash_func(core0_main)() 
{

	//pio_interrupt_clear(gg_capture_pio, 0);

	bool buttonsCleared = false;
	bool osdToggle = false;
	float backlight_level = 0;

	uint32_t time_now, gg_mode_switch_time = time_us_32();

	//wait for a vblank to start with, should be good after this, right?
	while(dma_channel_is_busy(dma_chan_fb1_write) && dma_channel_is_busy(dma_chan_fb2_write)) ;

	while(1) {

		time_now = time_us_32();

		//only switch mode evey 500ms to prevent too errattic switching
		if(last_gg != gg_now && gg_mode_switch_time + 500000 < time_now) 
		{
			dma_channel_abort(dma_chan_fb1_write);
			dma_channel_abort(dma_chan_fb1_reset);
			dma_channel_abort(dma_chan_fb2_write);
			dma_channel_abort(dma_chan_fb2_reset);
			
			sleep_ms(100);

			config_pios();	
			config_dma();		

			dma_channel_start(dma_chan_fb1_write);

			gg_mode_switch_time = time_us_32();			
			last_gg = gg_now;
		}


		//Scanning a frame out into the LCD is faster than ~16ms
		//So, we wait until the console starts sending out a new frame before we update the LCD again
		//If neither of these two DMA channels is busy, then the console must be in vblank
		//while(!dma_channel_is_busy(dma_chan_fb1_write) && !dma_channel_is_busy(dma_chan_fb2_write)) ;

		//See which of the two framebuffer is currently being written to and pick the other one to send to the LCD
		//This introduces a single frame of latency
		//while(dma_channel_is_busy(dma_chan_fb1_write) && dma_channel_is_busy(dma_chan_fb2_write)) ;

		//update lcd after 15ms for just over 60fps
		//if(start - last_frame_time > 14000)
		//if(start - last_frame_time > 12000)
		{
			
			//if(pio_interrupt_get(gg_capture_pio, 0))
			{
				//pio_interrupt_clear(gg_capture_pio, 0);

				if(dma_channel_is_busy(dma_chan_fb1_write)) framebuffer_to_use = framebuffer2;
				else framebuffer_to_use = framebuffer;
				
				if(osdToggle)
				{
					draw_overlay(framebuffer_to_use, 1000000 / (last_frame_start - last_frame_time)); //fps seems wrong
				}
				
				if(last_gg) 
				{
					update_lcd_gg(framebuffer_to_use);
				}
				else 
				{
					update_lcd_sms(framebuffer_to_use);
				}

				last_frame_time = last_frame_start;
				last_frame_start = time_us_32();
				
#ifdef DEBUG
				uint32_t end = time_us_32();
				
				printf("Rendering time: %i us, time since last: %i us\n", end - start, start - last_frame_time); 
#endif

				//doesn't need to happen at the start of the frame just needs to update once per frame
				read_in_spi();

				if(osdToggle && gg_btn1_pressed && menuItemIndex == 0)
				{
					osdToggle = false;
				}

				//wait until start+1+2 have been cleared before checking if menu needs toggling
				if(!buttonsCleared && !gg_start_now && !gg_btn1_now && !gg_btn2_now) buttonsCleared = true;
				
				//toggle on screen menu display
				if(buttonsCleared && gg_start_now && gg_btn1_now && gg_btn2_now) osdToggle = true;

				//update the brightness level
				backlight_level = brightness / 4096.f;
				backlight_level *= brightness_factor;
				set_backlight((uint16_t)((backlight_level * (pwm_wrap_target - pwm_min_level)) + pwm_min_level ));

			}
		}

	}

}


//
//secondary dvi output loop
//
void core1_main() 
{
	
	//
	//DVI INIT
	//
	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = pico_gg_lcd_conf;
	dvi0.vertical_repeat = DVI_VERTICAL_REPEAT_DEFAULT;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());
	
	//DVI AUDIO
	
 	for(uint32_t i = 0; i < DVI_AUDIO_BUFFER_SIZE; i++)
	{
		dvi_audio_buffer[i].channels[0] = 0;
		dvi_audio_buffer[i].channels[1] = 0;
	}		

	dvi_get_blank_settings(&dvi0)->top = 0;
	dvi_get_blank_settings(&dvi0)->bottom = 0;
	dvi_audio_sample_buffer_set(&dvi0, dvi_audio_buffer, DVI_AUDIO_BUFFER_SIZE);
	dvi_set_audio_freq(&dvi0, AUDIO_SAMPLE_RATE, DVI_AUDIO_CTS, 6272);


	//configure audio events on current core
	config_audio();


	while(true)
	{
		
		if(last_gg)
		{
			printf("starting DVI in GG mode\n");
			dvi0.vertical_repeat = DVI_VERTICAL_REPEAT_GG;
			dvi_register_irqs_this_core(&dvi0, DVI_DMA_IRQ);
			dvi_start(&dvi0);
			dvi_scanbuf_main_12bpp_noqueue_gg(&dvi0, framebuffer, framebuffer2, dma_chan_fb1_write, dma_chan_fb2_write);
			printf("stopping DVI in GG mode\n");
		}
		else
		{
			printf("starting DVI in SMS mode\n");
			dvi0.vertical_repeat = DVI_VERTICAL_REPEAT_SMS;
			dvi_register_irqs_this_core(&dvi0, DVI_DMA_IRQ);
			dvi_start(&dvi0);
			dvi_scanbuf_main_12bpp_noqueue_sms(&dvi0, framebuffer, framebuffer2, dma_chan_fb1_write, dma_chan_fb2_write);
			printf("stopping DVI in SMS mode\n");
		}
		dvi_stop(&dvi0);
	}

}

void init()
{

	startup_time = time_us_32();

	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	stdio_init_all();

	sleep_ms(100);
	printf("PICO GG LCD\n");

	gpio_init_mask(0b111111111111111111111111111111);
	gpio_set_dir(lcd_rst, GPIO_OUT);
	gpio_set_dir(lcd_den, GPIO_OUT);
	
	//enable the backlight supply from the booster
	gpio_init(lcd_dim);
	gpio_set_dir(lcd_dim, GPIO_OUT);
	gpio_put(lcd_dim, 1);

	uint8_t pwm_backlight_slice = config_backlight_pwm();

	//These functions MUST be called before dvi_init, since they unclaim all DMA channels and clear PIO memory
	init_lcd();

	config_in_spi();
	read_in_spi(); //read sms/gg mode state first
	last_gg = gg_now;

	claim_pio_sm();
	config_pios();

	claim_dma();
	config_dma();

	//config_interp();
	
	//changes strength of hdmi pins - didn't seem to help with a 20m cable which worked on some tvs
/*	
	for(int pin = 0; pin < 8; pin++)
	{
		gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
	}
*/
	gpio_put(lcd_den, 0);
	gpio_put(lcd_clk, 0);

	fill_framebuffer_with_test_pattern();
	//fill_framebuffer_with_test_pattern2();

	dma_channel_start(dma_chan_fb1_write);

}


int main() 
{
	init();

	multicore_reset_core1();

	multicore_launch_core1(core1_main); //libdvi core

	core0_main(); //main lcd core

	return 0;
}
 
