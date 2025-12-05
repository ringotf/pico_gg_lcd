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
#include "hardware/interp.h"

#include "../build/gg_capture.pio.h"
#include "../build/lcd_send.pio.h"
#include "../build/lcd_send_2x.pio.h"
#include "../build/lcd_send_4x.pio.h"
#include "../build/lcd_send_spi.pio.h"

#include "libdvi/dvi.h"
#include "libdvi/dvi_serialiser.h"
#include "libdvi/dvi_timing.h"

#include "tusb.h"

#define DVI_TIMING dvi_timing_640x480p_60hz

#define led_pin 25 //not used?

//#define gg_BTN1_pin 24 //11 	//not used?
//#define gg_BTN2_pin 23 //12 		//not used?
//#define gg_START_pin 22 	//not used?

#define gg_SMS_pin 25


#define gg_D1_pin 18
#define gg_D2_pin 19
#define gg_D3_pin 20
#define gg_D4_pin 21

#define gg_dw_pin 22
#define gg_cl2_pin 23
#define gg_clk_pin 24

#define pixels_in_scanline 320 //280 //300
#define scanlines_in_active_area 192 //144 //192

#define gg_pixel_width 160
#define gg_pixel_height 144


#define scanlines_in_active_area_min 100

#define FRAME_SIZE (pixels_in_scanline * scanlines_in_active_area * 2)
#define FRAME_SIZE_HALF (pixels_in_scanline * scanlines_in_active_area)

#define FRAME_SIZE_MIN (pixels_in_scanline * scanlines_in_active_area_min * 2)


#define lcd_rst 0 //17 //6

#define lcd_spi_latch 1 
#define lcd_spi_clk 2 
#define lcd_spi_mosi_1 3 
#define lcd_spi_mosi_2 4 
#define lcd_spi_mosi_3 5 

//#define lcd_hsync 13
#define lcd_vsync 14
#define lcd_clk 15
#define lcd_den 16

#define lcd_backlight 17


#define backlight_fdbck 26 //28 - swapped with lcd_den

#define brightness_pot 27 //29 - moved down 2 pins


#define lcd_pio_hscale 4
#define lcd_hscale_factor 1 / lcd_pio_hscale

#define lcd_target_width 640
#define lcd_send_width lcd_target_width * lcd_hscale_factor
#define lcd_channels 1 //3

//add extra pixels to the h porch
#define lcd_hblank_sync_len 2 * lcd_hscale_factor
#define lcd_hblank_front_len 44 * lcd_hscale_factor
#define lcd_hblank_back_len 42 * lcd_hscale_factor
#define lcd_hblank_len lcd_hblank_front_len + lcd_send_width + lcd_hblank_back_len


#define lcd_active_lines 160
#define lcd_vscale_factor 3

//add 240 extra lines to the v porch

#define lcd_vblank_sync_lines 2 
#define lcd_vblank_front_lines 16 
#define lcd_vblank_back_lines 14

#define scanlines_to_skip 1 //11

#define pixels_to_skip  (pixels_in_scanline * scanlines_to_skip)

struct dvi_inst dvi0;

static const struct dvi_serialiser_cfg pico_gg_lcd_conf = {
	.pio = pio1,
	.sm_tmds = {28, 2, 0},
	.pins_tmds = {28, 2, 0},
	.pins_clk = 4,
	.invert_diffpairs = false
};


//Use two framebuffers to prevent tearing
uint16_t * framebuffer = (uint16_t *)(0x20000000 + (1024 * 20));
uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 20) + (pixels_in_scanline * scanlines_in_active_area * 2));
//uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 30) + (pixels_in_scanline * scanlines_in_active_area * 2) + (pixels_in_scanline * 8));

uint16_t send_buffer[512];


uint32_t pot_history[16];
uint32_t brightness = 0;
uint8_t pwm_backlight_slice;

#define pwm_wrap_target  		((DVI_TIMING.bit_clk_khz * 1000) / 30000)
#define pwm_max_duty 			pwm_wrap_target  * 0.9f
#define pwm_feedback_target 	1.82f

uint32_t is_gg;
uint32_t gg_now;
uint32_t last_gg;

uint32_t dma_chan0;
uint32_t dma_chan1;
//uint32_t dma_chan2;
//uint32_t dma_chan3;
uint32_t dma_chan4;
uint32_t dma_chan5;

uint32_t last_frame_sent;



static inline __attribute__ ((always_inline)) void send_lcd_pixel(uint16_t data, bool hsync, bool vsync)
{
	//pio_sm_put(pio1, 0, data);
	//bit shifting is just because the shift register pins are wired MSB first so RGB but the lcd pins are aligned LSB first so BGR
	//change the shift register to lcd pin order and the shifting isnt needed
	//but the shifting doesnt seem to take much time anyway
	pio_sm_put(pio1, 0, (uint32_t)data << 20 );

	//pio_sm_put_blocking(pio1, 0, (uint32_t)data << 20 );
	//pio_sm_put_blocking(pio1, 0, (uint32_t)data << 18 | hsync << 30 | vsync << 31 );
	//pio_sm_put_blocking(pio1, 0, (uint32_t)data << 18 | 0 << 30 | 0 << 31 );
	//pio_sm_put_blocking(pio1, 0, data);

	//send_to_shift_dma(data, 2);
}

static inline __attribute__ ((always_inline)) void send_4blank_lcd_pixel()
{
	
	while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

	pio_sm_put(pio1, 0, 0);
	pio_sm_put(pio1, 0, 0);
	pio_sm_put(pio1, 0, 0);
	pio_sm_put(pio1, 0, 0);

	/*
	pio_sm_put_blocking(pio1, 0, 0);
	pio_sm_put_blocking(pio1, 0, 0);
	pio_sm_put_blocking(pio1, 0, 0);
	pio_sm_put_blocking(pio1, 0, 0);
	*/

}

static inline __attribute__ ((always_inline)) uint32_t unpack(uint32_t rgb_value) {
	uint32_t temp = rgb_value >> 8;
	temp |= rgb_value << 20;
	temp |= rgb_value << 6;
	temp &= 0b111100000011110000001111;

	return temp;
}

__attribute__ ((long_call, section (".time_critical"))) void update_lcd_gg() {	
	//Scanning a frame out into the LCD is faster than ~16ms
	//So, we wait until the console starts sending out a new frame before we update the LCD again
	//If neither of these two DMA channels is busy, then the console must be in vblank
	//while(!dma_channel_is_busy(dma_chan0) && !dma_channel_is_busy(dma_chan4)) ;

	uint16_t * curr_framebuffer;

	uint16_t spi_buffer[4] = {0xFFF, 0xFFF, 0xFFF, 0xFFF};  

	//See which of the two framebuffer is currently being written to and pick the other one to send to the LCD
	//This introduces a single frame of latency
	//if(dma_channel_is_busy(dma_chan0)) curr_framebuffer = framebuffer2;
	//else curr_framebuffer = framebuffer;

	curr_framebuffer = framebuffer;

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + 1 - 1);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_in_scanline * 1;
	dma_hw->ch[3].transfer_count = pixels_in_scanline * 1;

	//Because of the LCD's orientation, we actually need to scan the image out upside down and flipped
	//base points to the bottom right of the original image
	uint32_t base = pixels_in_scanline + 48 - 52 + (pixels_in_scanline * scanlines_in_active_area) - pixels_in_scanline * 51;

		
	static bool hsync = 0;
	static bool vsync = 0;

	gpio_put(lcd_den, 0);

	//start vsync pulse
	gpio_put(lcd_vsync, vsync);

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_sync_lines; wait_line ++) {
		
		hsync = 0;
		//gpio_put(lcd_hsync, hsync);
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
			//pio_sm_put_blocking(pio1, 0, 0);
			//send_lcd_pixel(0, hsync, vsync);
			send_4blank_lcd_pixel();
		}

		hsync = 1;
		//gpio_put(lcd_hsync, hsync);
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			//pio_sm_put_blocking(pio1, 0, 0);
			//send_lcd_pixel(0, hsync, vsync);
			send_4blank_lcd_pixel();
		}

	}
	//end vsync pulse

	//start vsync back porch
	vsync = 1;
	gpio_put(lcd_vsync, vsync);

	
	for(uint32_t wait_line = 0; wait_line < lcd_vblank_back_lines; wait_line ++) {
		
		hsync = 0;
		//gpio_put(lcd_hsync, 0);
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
			//pio_sm_put_blocking(pio1, 0, 0);
			//send_lcd_pixel(0, hsync, vsync);
			send_4blank_lcd_pixel();
		}

		hsync = 1;
		//gpio_put(lcd_hsync, 1);
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			//pio_sm_put_blocking(pio1, 0, 0);
			//send_lcd_pixel(0, hsync, vsync);
			send_4blank_lcd_pixel();
		}

	}
	//end vsync back porch


	//frame header
//do some blank header lines....

	static uint16_t line_diff = lcd_active_lines - gg_pixel_height;
	uint16_t header_count = line_diff * 0.5;
	uint16_t footer_count = line_diff - header_count;

	uint32_t y_base;
	uint32_t y_base_offset = (pixels_in_scanline - gg_pixel_width) * 0.5;
	uint32_t y_target;


	for (uint32_t y = 0; y < header_count ; y++)
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{

			hsync = 0;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait +=4) {
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);				
				send_4blank_lcd_pixel();
			}

			hsync = 1;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait +=4) {
				//pio_sm_put_blocking(pio1, 0, 0);				
				send_4blank_lcd_pixel();
			}
			

			//repeat the bottom lines at the top
			y_base = (gg_pixel_height + y) * pixels_in_scanline;			
			y_base += y_base_offset;
			y_target = y_base + lcd_send_width;
			
			gpio_put(lcd_den, 1);
			
			for (uint32_t x = y_base; x < y_target; x +=4) {			
				while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);
				//send_lcd_pixel(0, hsync, vsync);
				//send_lcd_pixel(0, hsync, vsync);
				//send_lcd_pixel(0, hsync, vsync);
				
				send_lcd_pixel(curr_framebuffer[x], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+1], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+2], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+3], hsync, vsync);
			}
			

			//Signal end of active scanline portion
			gpio_put(lcd_den, 0);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait +=4) {
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);						
				send_4blank_lcd_pixel();
			}
			
			hsync = 0;

		}
	}



	//frame body
	//start frame data
	//for (uint32_t y = 0; y < lcd_active_lines; y++)
	for (uint32_t y = 0; y < gg_pixel_height + footer_count; y++)	
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{
			hsync = 0;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait +=4) {
				
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);
				send_4blank_lcd_pixel();
			}

			hsync = 1;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait +=4) {
				
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);
				send_4blank_lcd_pixel();
			}

			//Where in the framebuffer the current scanline starts
			//uint32_t y_base = base - ((y * 310) >> 9) * pixels_in_scanline + 256;

			y_base = y * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			//y_base += (pixels_in_scanline - lcd_send_width)*0.5;
			y_base += y_base_offset;

			
			//while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

			//Tell LCD we're starting the active portion of the scanline
			gpio_put(lcd_den, 1);
			

			///unpack transforms 0b0000rrrrggggbbbb word into 0bbbb000000gggg000000rrrr word
			//Channel order is changed to bgr because that's what the LCD expects
			//Channel bits are spread out so we can do SIMD-within-a-word
			//The interpolator will do linear interpolation on all three channels at once, using a single 32-bit word
			//uint32_t framebuffer_pix_value = curr_framebuffer[y_base] & 4095;
			//uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);
			
			for (uint32_t x = y_base; x < y_base + lcd_send_width; x +=4)
			{

				/*
				uint32_t sub_pixel = (x * 133);
				uint32_t pixIndex = y_base - (sub_pixel >> 8);
				uint32_t pix_value = unpack(curr_framebuffer[pixIndex] & 4095);
				*/

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
				while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

				//while(pio_sm_is_tx_fifo_full(pio1, 0)) ;

				//This way, we only have to poll on the FIFO status once and then can just blast the three words at once
				//Seems to be significantly faster than pio_sm_put_blocking on each word
				//pio_sm_put(pio1, 0, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(pio1, 0, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(pio1, 0, pix_value & 15);


				//for(uint32_t z = 0; z < lcd_pio_hscale; z++)
				//{
					//pio_sm_put_blocking(pio1, 0, curr_framebuffer[x + z] & 4095);
				//	pio_sm_put(pio1, 0, curr_framebuffer[x + z] & 4095);
				//}
				
				//pio_sm_put(pio1, 0, curr_framebuffer[x] & 4095);
				//pio_sm_put(pio1, 0, curr_framebuffer[x+1] & 4095);

				//fill the fifo - does not correlate to lcd_pio_hscale. the fifo is just 4 words max
				//pio_sm_put(pio1, 0, curr_framebuffer[x] & 4095);
				//pio_sm_put(pio1, 0, curr_framebuffer[x+1] & 4095);
				//pio_sm_put(pio1, 0, curr_framebuffer[x+2] & 4095);
				//pio_sm_put(pio1, 0, curr_framebuffer[x+3] & 4095);
				
				/*
				pio_sm_put_blocking(pio1, 0, curr_framebuffer[x] & 4095);
				pio_sm_put_blocking(pio1, 0, curr_framebuffer[x+1] & 4095);
				pio_sm_put_blocking(pio1, 0, curr_framebuffer[x+2] & 4095);
				pio_sm_put_blocking(pio1, 0, curr_framebuffer[x+3] & 4095);
				*/

				send_lcd_pixel(curr_framebuffer[x], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+1], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+2], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+3], hsync, vsync);

/*
				send_lcd_pixel(spi_buffer[0], hsync, vsync);
				send_lcd_pixel(spi_buffer[1], hsync, vsync);
				send_lcd_pixel(spi_buffer[2], hsync, vsync);
				send_lcd_pixel(spi_buffer[3], hsync, vsync);
	*/			
			}


			//while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

			//Signal end of active scanline portion
			gpio_put(lcd_den, 0);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait +=4) {

				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);
				send_4blank_lcd_pixel();
			}
			
			hsync = 0;
			//gpio_put(lcd_hsync, hsync);
		}

	}




/*
	//frame footer
//do some blank footer lines....
	for (uint32_t y = gg_pixel_height; y < gg_pixel_height + footer_count; y++)
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{

			hsync = 0;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait ++) {
				//pio_sm_put_blocking(pio1, 0, 0);
				send_lcd_pixel(0, hsync, vsync);
			}

			hsync = 1;
			//gpio_put(lcd_hsync, hsync);

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait ++) {
				//pio_sm_put_blocking(pio1, 0, 0);
				send_lcd_pixel(0, hsync, vsync);
			}

			gpio_put(lcd_den, 1);

			y_base = y * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			y_base += y_base_offset;

			//blank line

			for(uint32_t x = y_base; x < y_base + lcd_send_width; x +=4) {
				//pio_sm_put_blocking(pio1, 0, 0);
				//send_lcd_pixel(0, hsync, vsync);
				
				send_lcd_pixel(curr_framebuffer[x], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+1], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+2], hsync, vsync);
				send_lcd_pixel(curr_framebuffer[x+3], hsync, vsync);
			}
			


			//Signal end of active scanline portion
			gpio_put(lcd_den, 0);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait ++) {
				//pio_sm_put_blocking(pio1, 0, 0);
				send_lcd_pixel(0, hsync, vsync);
			}
			
			hsync = 0;

		}
	}
*/


	//end frame data

	//start vsync front porch

	//Start of vblank 
	gpio_put(lcd_den, 0);

	hsync = 0;
	//Send empty data for the entirety of vblank
	//gpio_put(lcd_hsync, hsync);
	 
	for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
		//pio_sm_put_blocking(pio1, 0, 0);
		//send_lcd_pixel(0, hsync, vsync);
		send_4blank_lcd_pixel();
	}

	hsync = 1;
	//gpio_put(lcd_hsync, hsync);

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_front_lines; wait_line ++) {
		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			//pio_sm_put_blocking(pio1, 0, 0);
			//send_lcd_pixel(0, hsync, vsync);
			send_4blank_lcd_pixel();
		}

		/*for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}*/
	}

	//end vsync front porch
	
	hsync = 0;
	vsync = 0;
	//gpio_put(lcd_hsync, hsync);
	gpio_put(lcd_vsync, vsync);
	
	gpio_put(lcd_den, 0);
}

//The only thing that changes for SMS mode are the scaling constants
//However, putting those values in variables considerably slows down this function
//So, instead, we use magic constants baked into the code
__attribute__ ((long_call, section (".time_critical"))) void update_lcd_sms() {	
    while(!dma_channel_is_busy(dma_chan0) && !dma_channel_is_busy(dma_chan4)) ;
	uint16_t * curr_framebuffer;

	if(dma_channel_is_busy(dma_chan0)) curr_framebuffer = framebuffer2;
	else curr_framebuffer = framebuffer;

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + scanlines_to_skip - 1);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_in_scanline * scanlines_to_skip;
	dma_hw->ch[3].transfer_count = pixels_in_scanline * scanlines_to_skip;

	uint32_t line_counter = 0;
	uint32_t base = pixels_in_scanline + 256 + 48 - 3 + (pixels_in_scanline * scanlines_in_active_area) - pixels_in_scanline * 2;

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

			while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
		}

		gpio_put(lcd_den, 1);

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}

	gpio_put(lcd_den, 1);

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_back_lines; wait_line ++) {
		for(uint32_t pixel = 0; pixel < lcd_send_width; pixel ++) {
			for(uint32_t channel = 0; channel < lcd_channels; channel ++) {
				pio_sm_put_blocking(pio1, 0, 0);
			}
		}

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}
}

void config_pios() {
	pio_clear_instruction_memory(pio1);

	//PIO1 is used to send data to the LCD without hogging the CPU
	pio_sm_claim(pio1, 0);

	//pio1 sm1 used for lcd clk
	pio_sm_claim(pio1, 1);

/*
	uint8_t lcd_send_1x = pio_add_program(pio1, &lcd_send_program);
	pio_sm_config lcd_send_1x_config = lcd_send_program_get_default_config(lcd_send_1x);

	uint8_t lcd_send_2x = pio_add_program(pio1, &lcd_send_2x_program);
	pio_sm_config lcd_send_2x_config = lcd_send_2x_program_get_default_config(lcd_send_2x);	

	uint8_t lcd_send_4x = pio_add_program(pio1, &lcd_send_4x_program);
	pio_sm_config lcd_send_4x_config = lcd_send_4x_program_get_default_config(lcd_send_4x);
*/

	uint8_t lcd_send_spi = pio_add_program(pio1, &lcd_send_spi_program);
	pio_sm_config lcd_send_spi_config = lcd_send_spi_program_get_default_config(lcd_send_spi);

	uint8_t lcd_send_clk = pio_add_program(pio1, &lcd_send_clk_program);
	pio_sm_config lcd_send_clk_config = lcd_send_clk_program_get_default_config(lcd_send_clk);


	uint8_t lcd_send = lcd_send_spi;
	pio_sm_config lcd_send_config = lcd_send_spi_config;

	/*
	switch(lcd_pio_hscale)
	{
		case 2:
			lcd_send = lcd_send_2x;
			lcd_send_config = lcd_send_2x_config;
			break;

		case 4:
			lcd_send = lcd_send_4x;
			lcd_send_config = lcd_send_4x_config;
			break;

		default:
			lcd_send = lcd_send_1x;
			lcd_send_config = lcd_send_1x_config;
			break;
	}
	*/

	//uint8_t lcd_send = lcd_send_2x;
	//pio_sm_config lcd_send_config = lcd_send_2x_config;

	sm_config_set_clkdiv(&lcd_send_config, 1);
	sm_config_set_out_pins(&lcd_send_config, lcd_spi_mosi_1, 3);
	//sm_config_set_sideset_pins(&lcd_send_config, lcd_spi_latch);
	sm_config_set_sideset_pins(&lcd_send_config, lcd_spi_clk);
	sm_config_set_set_pins(&lcd_send_config, lcd_spi_latch, 1);
	//sm_config_set_set_pins(&lcd_send_config, lcd_clk, 1);

	sm_config_set_out_shift(&lcd_send_config, false, true, 12);

	sm_config_set_clkdiv(&lcd_send_clk_config, 1);
	sm_config_set_set_pins(&lcd_send_clk_config, lcd_clk, 1);


	pio_gpio_init(pio1, lcd_spi_latch);
	pio_gpio_init(pio1, lcd_spi_clk);
	pio_gpio_init(pio1, lcd_spi_mosi_1);
	pio_gpio_init(pio1, lcd_spi_mosi_2);
	pio_gpio_init(pio1, lcd_spi_mosi_3);
	
	pio_gpio_init(pio1, lcd_clk);

	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_spi_latch), (1 << lcd_spi_latch) );
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_spi_mosi_1), (1 << lcd_spi_mosi_1) );
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_spi_mosi_2), (1 << lcd_spi_mosi_2) );
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_spi_mosi_3), (1 << lcd_spi_mosi_3) );
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_spi_clk), (1 << lcd_spi_clk) );

	pio_sm_set_pindirs_with_mask(pio1, 1, (1 << lcd_clk), (1 << lcd_clk) );




	pio_sm_init(pio1, 0, lcd_send, &lcd_send_config);
	pio_sm_set_enabled(pio1, 0, true);

	pio_sm_init(pio1, 1, lcd_send_clk, &lcd_send_clk_config);
	pio_sm_set_enabled(pio1, 1, true);




	pio_clear_instruction_memory(pio0);

	//PIO0 captures data from the GG's video bus
	pio_sm_claim(pio0, 0);
	pio_sm_claim(pio0, 1);
	pio_sm_claim(pio0, 2);

	uint8_t detect_vblank = pio_add_program(pio0, &detect_vblank_program);
	pio_sm_config detect_vblank_config = detect_vblank_program_get_default_config(detect_vblank);
	sm_config_set_clkdiv(&detect_vblank_config, 1);

	uint8_t detect_hblank = pio_add_program(pio0, &detect_hblank_program);
	pio_sm_config detect_hblank_config = detect_hblank_program_get_default_config(detect_hblank);
	sm_config_set_clkdiv(&detect_hblank_config, 1);

	uint8_t get_data = pio_add_program(pio0, &get_data_program);
	pio_sm_config get_data_config = get_data_program_get_default_config(get_data);
	sm_config_set_clkdiv(&get_data_config, 1);
	sm_config_set_in_pins(&get_data_config, gg_D1_pin);
	sm_config_set_in_shift(&get_data_config, false, false, 32);
	//sm_config_set_jmp_pin(&get_data_config, gg_clk_pin);

	pio_sm_init(pio0, 0, detect_vblank, &detect_vblank_config);
	pio_sm_init(pio0, 1, detect_hblank, &detect_hblank_config);
	pio_sm_init(pio0, 2, get_data, &get_data_config);

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + scanlines_to_skip - 1);
	//pio_sm_put_blocking(pio0, 1, 140);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	pio_sm_put_blocking(pio0, 2, pixels_in_scanline - 1); 
	//pio_sm_put_blocking(pio0, 2, 160); 
	pio_sm_exec(pio0, 2, pio_encode_pull(false, true));

	pio_enable_sm_mask_in_sync(pio0, 0b111);
}

uint32_t dummy;

void config_dma() {
	for(uint32_t c = 0; c < 12; c++) {
		dma_channel_cleanup(c);
    	dma_channel_unclaim(c);
	}

	//DVI code uses claim_unused_channel, so we must as well, instead of explicitly picking DMA channels
	dma_chan0 = dma_claim_unused_channel(true);
	dma_chan1 = dma_claim_unused_channel(true);
	//dma_chan2 = dma_claim_unused_channel(true);
	//dma_chan3 = dma_claim_unused_channel(true);
	dma_chan4 = dma_claim_unused_channel(true);
	dma_chan5 = dma_claim_unused_channel(true);

/*
	//PIO0 SM2 is the one that actually sends out pixel data
	dma_channel_config g = dma_channel_get_default_config(dma_chan2);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, dma_chan0);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan2,
			&g,
			&dummy,
			&pio0_hw->rxf[2],
			//The first few active display scanlines are actually blank
			//We skip them by taking data from the PIO and discarding it into the dummy variable
			//We than chain to dma_chan0 
			pixels_to_skip,
			false);
*/

	//Now we can save the scanlines that actually have image data into the framebuffer
	dma_channel_config f = dma_channel_get_default_config(dma_chan0);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan1);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan0,
			&f,
			//Note that we're filling the first framebuffer here
			//There's two of them, for double buffering, so we don't get screen tearing
			&framebuffer[0],
			&pio0_hw->rxf[2],
			//Again, the active area has border scanlines that don't actually contain game graphics
			//We only save the *real* scanlines into the framebuffer
			//Then chain to dma_chan1
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan0's write address register to point to the start of framebuffer0
	//Then chain to dma_chan3
	dma_channel_config e = dma_channel_get_default_config(dma_chan1);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	//channel_config_set_chain_to(&e, dma_chan3);
	channel_config_set_chain_to(&e, dma_chan4);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan1,
			&e,
			&(dma_hw->ch[dma_chan0].write_addr),
			&framebuffer,
			1,
			false);

/*
	//Skip unwanted lines, but now for the second frame
	g = dma_channel_get_default_config(dma_chan3);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, dma_chan4);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan3,
			&g,
			&dummy,
			&pio0_hw->rxf[2],
			pixels_to_skip,
			false);
*/

	//Save active scanlines into second framebuffer
	f = dma_channel_get_default_config(dma_chan4);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan5);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan4,
			&f,
			&framebuffer2[0],
			&pio0_hw->rxf[2],
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan4's write address and chain into dma_chan2 to restart the whole DMA chain
	e = dma_channel_get_default_config(dma_chan5);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	//channel_config_set_chain_to(&e, dma_chan2);
	channel_config_set_chain_to(&e, dma_chan0);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan5,
			&e,
			&(dma_hw->ch[dma_chan4].write_addr),
			&framebuffer2,
			1,
			false);
}

void config_backlight_supply(uint8_t pwm_backlight_slice) {
	adc_init();
	adc_gpio_init(backlight_fdbck);
	adc_select_input(backlight_fdbck - ADC_BASE_PIN);
	//adc_run(true);
	//adc_fifo_setup(true, false, 0, 0, 0);

	pwm_hw->slice[pwm_backlight_slice].cc = 250;
	sleep_ms(50);

	float tensao_media = 0;

	/*while(tensao_media < 19 || tensao_media > 20) {
		for(uint32_t m = 0; m < 5; m++) {
			sleep_ms(2);
			tensao_media += adc_fifo_get();
		}

		tensao_media *= 3.3;
		tensao_media *= 11;
		tensao_media /= 5;
		tensao_media /= 4095;

		if(tensao_media < 10) {
			pwm_hw->slice[pwm_backlight_slice].cc += 50;
		}
		else if(tensao_media < 15) {
			pwm_hw->slice[pwm_backlight_slice].cc += 20;
		}
		else if(tensao_media < 19) {
			pwm_hw->slice[pwm_backlight_slice].cc += 10;
		}
		else if(tensao_media > 21) {
			pwm_hw->slice[pwm_backlight_slice].cc -= 20;
		}
		else if(tensao_media > 19) {
			pwm_hw->slice[pwm_backlight_slice].cc -= 10;
		}

		gpio_put(led_pin, !gpio_get(led_pin));
	}*/
	pwm_hw->slice[pwm_backlight_slice].cc = 500;
	//pwm_hw->slice[pwm_backlight_slice].cc = 20;
}

uint8_t config_backlight_pwm() {
	gpio_set_function(lcd_backlight, GPIO_FUNC_PWM);
	pwm_backlight_slice = pwm_gpio_to_slice_num(lcd_backlight);

	pwm_config config = pwm_get_default_config();
	pwm_config_set_phase_correct(&config, false);	//default is false anyway
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	
	//uint16_t pwm_wrap_target = ((DVI_TIMING.bit_clk_khz * 1000) / 30000);
	pwm_config_set_wrap(&config, pwm_wrap_target);
	//pwm_config_set_wrap(&config, (DVI_TIMING.bit_clk_khz * 1000) / 30000);

	pwm_init(pwm_backlight_slice, &config, true);

	return pwm_backlight_slice;
}

void config_interp() {
	//Claim both lanes
	interp_claim_lane_mask(interp0, 0b11);

	interp_config cfg = interp_default_config();
	//Only interpolator 0 has " blend " mode, which does linear interpolation
	interp_config_set_blend(&cfg, true);
	interp_set_config(interp0, 0, &cfg);

	cfg = interp_default_config();
	interp_set_config(interp0, 1, &cfg);
}

void init_lcd() {
	sleep_ms(100);

	//gpio_put(lcd_rst, 1);
	sleep_ms(10);
	gpio_put(lcd_rst, 0);
	sleep_ms(10);
	gpio_put(lcd_rst, 1);

	sleep_ms(100);
}

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

			/*if(x > 150) 
			{
				pixel |= 15;
				pixel |= (15 << 8);
				pixel |= (15 << 4);
			}*/
			//if(y > 96) pixel |= (15 << 4);


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
			
			/*if(x > 150 && y > 96) pixel |= 15;
			if(x < 150 && y < 96) pixel |= 15;

			if(x < 150 && y > 96) pixel |= (15 << 8);
			if(x <= 150 && y <= 96) pixel |= (15 << 8);

			if(x > 150 && y < 96) pixel |= (15 << 4);
			if(x <= 150 && y <= 96) pixel |= (15 << 4);*/
			
			
			/*
			if(x < 150)
			{
				pixel |= 15;
			}
			
			if(y < 96)
			{
				pixel |= (15 << 8);
			}
*/
			

			//pixel = ((x + y) % 2) ? 0xF800 : 0x07E0; //Red or Green (RGB565)
			//pixel = ((x) % 2) ? 0xF800 : 0x07E0; //Red or Green (RGB565)
			//pixel = ((x/8) % 2) ? 0xF800 : 0x07E0; //Red or Green (RGB565)

			//test_frame_buffer[(y * pixels_in_scanline) + x] = pixel;

			//pixel = unpack(pixel);

			framebuffer[x + (y * pixels_in_scanline)]  = pixel;
			//framebuffer2[x + (y * pixels_in_scanline)]  = ~pixel;
			framebuffer2[x + (y * pixels_in_scanline)]  = pixel;

			//framebuffer[x + (y * pixels_in_scanline)] = pixel;
			//framebuffer2[x + (y * pixels_in_scanline)] = ~pixel;
		}
	}
}


void send_frame_over_usb() 
{	

	//if((time_us_32() - last_frame_sent > 500000) )
	{
		tud_task(); // Process USB events (non-blocking??)
		
		if(tud_cdc_connected() && tud_cdc_available() )
		{
			uint16_t * curr_framebuffer;

			//See which of the two framebuffer is currently being written to and pick the other one to send to the LCD
			if(!dma_channel_is_busy(dma_chan0)) 
			{
				curr_framebuffer = framebuffer2;
			}
			else 
			{
				curr_framebuffer = framebuffer;
			
				
				uint32_t frame_size_to_use = FRAME_SIZE * 2;//_MIN; //FRAME_SIZE_HALF;

				//frame_size_to_use -= pixels_in_scanline * 16; //51;
				
				//memcpy(send_buffer, curr_framebuffer, pixels_in_scanline  * 144); // Copy full buffer

				//char buf[64];
				//uint32_t count = tud_cdc_read(buf, sizeof(buf));
				tud_cdc_read_flush();
				//if(count > 0 && buf[0] == 0x0A) //enter received - send the current frame buffer
				{		

					uint32_t offset = 0;
					uint32_t CHUNK_SIZE = CFG_TUD_CDC_TX_BUFSIZE;

					while(offset < frame_size_to_use)
					{
						uint32_t chunk = (frame_size_to_use - offset < CHUNK_SIZE) ? frame_size_to_use - offset : CHUNK_SIZE;


						//tud_cdc_write(test_frame_buffer, sizeof(test_frame_buffer));
						//if(tud_cdc_write_available() >= (pixels_in_scanline * scanlines_in_active_area))
			//			if(tud_cdc_write_available() >= sizeof(framebuffer))
						if(tud_cdc_write_available() >= chunk)
						{

							//memcpy(send_buffer, (uint8_t *)curr_framebuffer + offset, chunk);
							//tud_cdc_write(send_buffer, chunk);

							tud_cdc_write((uint8_t*)curr_framebuffer + offset, chunk);

							offset += chunk;
							
							//tud_cdc_write(framebuffer, sizeof(framebuffer));
							//tud_cdc_write(framebuffer, (pixels_in_scanline * scanlines_in_active_area));

							//if(tud_cdc_write_available() >= sizeof(framebuffer))
							//if(tud_cdc_write_available() >= (pixels_in_scanline * scanlines_in_active_area) *2)
							/*{
								
								//fill_framebuffer_with_test_pattern();

							}*/

							tud_cdc_write_flush();
						}
						tud_task();
					}

					last_frame_sent = time_us_32();
				}
			}
		}

	}
}


void core1_main() 
{
	
	/*dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	dvi_scanbuf_main_12bpp(&dvi0);*/
	
	
	while(1)
	{
		//send_frame_over_usb();


 		adc_select_input(backlight_fdbck - ADC_BASE_PIN);

		uint16_t v_div = adc_read();
		float v_feedback = v_div * (pwm_feedback_target / 4095.f);
		float error = (pwm_feedback_target * 0.5f) - v_feedback;

		int32_t adjustment = (int32_t)(error * 0.05f * (float)pwm_wrap_target / (pwm_feedback_target));

		pwm_hw->slice[pwm_backlight_slice].cc += adjustment;

		if(pwm_hw->slice[pwm_backlight_slice].cc < 0)
			pwm_hw->slice[pwm_backlight_slice].cc = 0;
		
		if(pwm_hw->slice[pwm_backlight_slice].cc > pwm_max_duty) 
			pwm_hw ->slice[pwm_backlight_slice].cc = pwm_max_duty;


		//printf("VDiv: %.2f, Feedback: %.2fV, Error: %.2f, Adjustment: %i, Duty: %u\n", v_div,  v_feedback, error, adjustment, pwm_hw->slice[pwm_backlight_slice].cc);


		sleep_ms(15); 
	}

	//__builtin_unreachable();
}



int main() 
{
	vreg_set_voltage(VREG_VOLTAGE_1_10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	stdio_init_all();

	tusb_init(); //initialise TinyUSB stack

	gpio_init_mask(0b11111111111111111111111111111111);
	gpio_set_dir_out_masked(1 << led_pin);
	gpio_set_dir_out_masked(1 << lcd_rst);
	gpio_set_dir_out_masked(1 << lcd_den);
	//gpio_set_dir_out_masked(1 << lcd_hsync);
	gpio_set_dir_out_masked(1 << lcd_vsync);
	gpio_set_dir_out_masked(1 << lcd_backlight);
	
	uint8_t pwm_backlight_slice = config_backlight_pwm();
	config_backlight_supply(pwm_backlight_slice);

	//These functions MUST be called before dvi_init, since they unclaim all DMA channels and clear PIO memory
	init_lcd();
	config_pios();
	config_dma();
	//config_interp();


	//dvi0.timing = &DVI_TIMING;
	//dvi0.ser_cfg = pico_gg_lcd_conf;
	//dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	multicore_reset_core1();

	multicore_launch_core1(core1_main);

	gpio_put(lcd_den, 0);
	//gpio_put(lcd_hsync, 0);
	gpio_put(lcd_vsync, 0);
	gpio_put(led_pin, 1);
	
	gpio_put(lcd_clk, 0);
	gpio_put(lcd_backlight, 0);

	/*adc_init();
	adc_gpio_init(brightness_pot);
	adc_select_input(brightness_pot - ADC_BASE_PIN);
	adc_run(true);
	adc_fifo_setup(true, false, 0, 0, 0);*/

	fill_framebuffer_with_test_pattern();

	//dma_channel_start(dma_chan2);
	dma_channel_start(dma_chan0);

	uint32_t last_frame_time = 0;

	while(1) {
/*
		last_gg = gg_now;
		gg_now = !gpio_get(gg_SMS_pin);

		if(last_gg == gg_now) is_gg = gg_now;
*/
		uint32_t start = time_us_32();
		//update lcd after 15ms for just over 60fps
		if(start - last_frame_time > 15000)
		{
			update_lcd_gg();
			last_frame_time = start;
		}

		/*
		if(!is_gg) update_lcd_gg();
		else update_lcd_sms();
		uint32_t end = time_us_32();
		*/

		/*
		adc_select_input(brightness_pot - ADC_BASE_PIN);

		uint32_t avg = 0;
		for (uint32_t c = 15; c > 0; c--)
		{
			avg += pot_history[c];
			pot_history[c] = pot_history[c - 1];
		}
		pot_history[0] = adc_fifo_get();
		avg += pot_history[0];
		avg >>= 4;
		brightness = avg;
		brightness *= 6;
		*/

		//printf("Pot: %i\n", brightness);
		//printf("Rendering time: %i us\n", end - start); 

		//send_frame_over_usb();



	}

	return 0;
}
 
