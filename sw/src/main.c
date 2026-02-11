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
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "hardware/resets.h"
#include "pico/bootrom.h"

#include "main.h"
#include "../build/gg_capture.pio.h"
#include "../build/lcd_send.pio.h"
#include "../build/lcd_send_2x.pio.h"
#include "../build/lcd_send_4x.pio.h"
#include "../build/lcd_send_spi.pio.h"
#include "font8x8_basic.h"

#include "libdvi/dvi.h"
#include "libdvi/dvi_serialiser.h"
#include "libdvi/dvi_timing.h"

#include "tusb.h"

#define DVI_TIMING dvi_timing_640x480p_60hz
//#define DVI_TIMING dvi_timing_720x480p_60hz

#define led_pin 25 //not used?

//#define gg_BTN1_pin 24 //11 	//not used?
//#define gg_BTN2_pin 23 //12 		//not used?
//#define gg_START_pin 22 	//not used?

//#define gg_SMS_pin 10

#define gg_D1_pin 11 
#define gg_D2_pin 12 
#define gg_D3_pin 13 
#define gg_D4_pin 14 

#define gg_cl2_pin 15 
#define gg_dw_pin 16
#define gg_clk_pin 17

//need to move hdmi pins to free up adc... or use separate adc module
#define gg_audio_l_pin 27
#define gg_audio_r_pin 26
//#define gg_audio_pwr_pin 27 //3v for voltage divider for 1.65v bias in prototype

//#define pixels_in_scanline 256 //280 //300
//#define scanlines_in_active_area  192 //160 //144 //192


#define scanlines_in_active_area_min 100

#define FRAME_SIZE (pixels_in_scanline * scanlines_in_active_area * 2)
#define FRAME_SIZE_HALF (pixels_in_scanline * scanlines_in_active_area)

#define FRAME_SIZE_MIN (pixels_in_scanline * scanlines_in_active_area_min * 2)


#define lcd_spi_latch 18
#define lcd_spi_clk 19
#define lcd_spi_mosi_1 23
#define lcd_spi_mosi_2 24
#define lcd_spi_mosi_3 25

#define btn_sr_spi spi0
#define btn_sr_miso 20
#define btn_sr_load 21
#define btn_sr_clk 22

//#define lcd_hsync 13
//#define lcd_vsync 8 //14

#define lcd_rst 10
#define lcd_clk 9
#define lcd_den 8

//#define lcd_backlight 28

#define lcd_send_pio pio0 //pio1
#define lcd_send_sm 2

#define lcd_send_clk_pio pio0 //pio1
#define lcd_send_clk_sm 3 //1

//#define backlight_fdbck 28 //- swapped with lcd_den

#define gg_capture_pio pio0
//#define gg_capture_vblank_sm 0
#define gg_capture_hblank_sm 0 //1
#define gg_capture_getdata_sm 1 //2
//#define gg_capture_front_h_porch_sm 3

#define brightness_pot 28
#define lcd_dim 29


#define lcd_pio_hscale 4
#define lcd_hscale_factor 1 / lcd_pio_hscale

#define lcd_target_width 640
#define lcd_send_width 160 //lcd_target_width * lcd_hscale_factor
#define lcd_channels 1 //3

//add extra pixels to the h porch
#define lcd_hblank_sync_len 2 * lcd_hscale_factor // 2
#define lcd_hblank_front_len 44 * lcd_hscale_factor  //44 //24
#define lcd_hblank_back_len 42 * lcd_hscale_factor //42 //12
#define lcd_hblank_len lcd_hblank_front_len + lcd_send_width + lcd_hblank_back_len //this is sync plus both porches plus active display area


#define lcd_active_lines 160
#define lcd_vscale_factor 3

//add 240 extra lines to the v porch

#define lcd_vblank_sync_lines 2 //2
#define lcd_vblank_front_lines 16 // 22 //16
#define lcd_vblank_back_lines 14 //88 //14

#define dvi_pio pio1
#define dvi_tmds_sm_0 0
#define dvi_tmds_sm_1 1
#define dvi_tmds_sm_2 2

#define scanlines_to_skip 1 //11

#define pixels_to_skip  (pixels_in_scanline * scanlines_to_skip)

// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20

struct dvi_inst dvi0;

static struct dvi_serialiser_cfg pico_gg_lcd_conf = {
	.pio = dvi_pio,
	.sm_tmds = {dvi_tmds_sm_0, dvi_tmds_sm_1, dvi_tmds_sm_2},
	.pins_tmds = {2, 4, 6},
	//.pins_tmds = {24, 18, 20},
	.pins_clk = 0,
	//.pins_clk = 22,
	.invert_diffpairs = true
};

#define DVI_DMA_IRQ DMA_IRQ_0


//Use two framebuffers to prevent tearing
uint16_t * framebuffer = (uint16_t *)(0x20000000 + (1024 * 40));
uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 40) + (pixels_in_scanline * scanlines_in_active_area * 2));
//uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 30) + (pixels_in_scanline * scanlines_in_active_area * 2) + (pixels_in_scanline * 8));

//uint16_t send_buffer[512];

#define ADC_SAMPLE_MULTIPLIER 1
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_CHANNEL_COUNT 2
#define AUDIO_BUFFER_BITS 11
#define AUDIO_BUFFER_SIZE (1 << AUDIO_BUFFER_BITS)
//uint16_t AUDIO_BUFFER_SIZE = 1 << AUDIO_BUFFER_BITS;
//uint16_t AUDIO_BUFFER_SIZE = 1 << AUDIO_BUFFER_BITS;
//#define AUDIO_BUFFER_SIZE 1024
#define AUDIO_DMA_IRQ DMA_IRQ_1

__attribute__((aligned((1 << AUDIO_BUFFER_BITS))))
__attribute__((section(".time_critical.ram")))
static uint16_t audio_buffer[AUDIO_BUFFER_SIZE];
//volatile uint16_t audio_l_buffer[AUDIO_BUFFER_SIZE];
//volatile uint16_t audio_r_buffer[AUDIO_BUFFER_SIZE];

static const uint32_t audio_buffer_addr = (uint32_t)audio_buffer;


#define DVI_AUDIO_CTS 28000
#define DVI_AUDIO_BUFFER_SIZE 256
audio_sample_t dvi_audio_buffer[DVI_AUDIO_BUFFER_SIZE];


uint32_t pot_history[16];
uint32_t brightness = 0;
uint8_t pwm_backlight_slice;

#define pwm_wrap_target  		((DVI_TIMING.bit_clk_khz * 1000) / 30000)
#define pwm_max_duty 			pwm_wrap_target  * 0.9f
#define pwm_min_duty 			pwm_wrap_target  * 0.1f
#define pwm_feedback_target 	1.82f

uint32_t is_gg;
uint32_t gg_now;
uint32_t last_gg;

uint32_t gg_start_now;
uint32_t gg_btn1_now;
uint32_t gg_btn2_now;

uint32_t dma_chan_fb1_write;
uint32_t dma_chan_fb1_reset;
//uint32_t dma_chan2;
//uint32_t dma_chan3;
uint32_t dma_chan_fb2_write;
uint32_t dma_chan_fb2_reset;

uint32_t startup_time;
uint32_t last_frame_sent;

static inline __attribute__ ((always_inline)) void send_lcd_pixel(uint16_t data)
{
	
	//bit shifting is just because the shift register pins are wired MSB first so RGB but the lcd pins are aligned LSB first so BGR
	//change the shift register to lcd pin order and the shifting isnt needed
	//but the shifting doesnt seem to take much time anyway
	//pio_sm_put(lcd_send_pio, lcd_send_sm, (uint32_t)data << 20 );
	pio_sm_put(lcd_send_pio, lcd_send_sm, (uint32_t)data << 20 );

	//pio_sm_put(lcd_send_pio, lcd_send_sm, (uint32_t)data);

	//pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, (uint32_t)data << 20 );
	//pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, (uint32_t)data << 18 | hsync << 30 | vsync << 31 );
	//pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, (uint32_t)data << 18 | 0 << 30 | 0 << 31 );
	//pio_sm_put_blocking(lcd_send_pio, lcd_send_sm, data);

	//send_to_shift_dma(data, 2);
	
}

static inline __attribute__ ((always_inline)) void send_4blank_lcd_pixel()
{
	while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);
	pio_sm_put(lcd_send_pio, lcd_send_sm, 0);

}

volatile uint32_t for_nop_count = 0;

static inline __attribute__ ((always_inline)) void lcd_den_set(bool lcd_den_value, bool skip_interrupt) 
{
	//printf("lcd den set: %d pixel complete: %d pixel index: %d\n", lcd_den_value, pixel_complete, pixel_index);

	while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm));

	for(for_nop_count = 0; for_nop_count < 5; for_nop_count++)
	{
		__asm volatile("nop");
	}
	
	while(!skip_interrupt && !pio_interrupt_get(lcd_send_pio, 4));

	gpio_put(lcd_den, lcd_den_value);

	/*for(for_nop_count = 0; for_nop_count < 5; for_nop_count++)
	{
		__asm volatile("nop");
	}*/
	
}


static inline __attribute__ ((always_inline)) uint32_t unpack(uint32_t rgb_value) {
	uint32_t temp = rgb_value >> 8;
	temp |= rgb_value << 20;
	temp |= rgb_value << 6;
	temp &= 0b111100000011110000001111;

	return temp;
}


uint16_t * framebuffer_to_use;

__attribute__ ((long_call, section (".time_critical"))) void update_lcd_gg(uint16_t * curr_framebuffer) {	
	//Scanning a frame out into the LCD is faster than ~16ms
	//So, we wait until the console starts sending out a new frame before we update the LCD again
	//If neither of these two DMA channels is busy, then the console must be in vblank
	//while(!dma_channel_is_busy(dma_chan_fb1_write) && !dma_channel_is_busy(dma_chan_fb2_write)) ;

	uint16_t spi_buffer[4] = {0x800, 0x800, 0x800, 0x800};  

	//See which of the two framebuffer is currently being written to and pick the other one to send to the LCD
	//This introduces a single frame of latency
	//if(dma_channel_is_busy(dma_chan_fb1_write)) curr_framebuffer = framebuffer2;
	//else curr_framebuffer = framebuffer;

	//curr_framebuffer = framebuffer;

	//pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, scanlines_in_active_area + 1 - 1);
	//pio_sm_exec(gg_capture_pio, gg_capture_hblank_sm, pio_encode_pull(false, true));

	//dma_hw->ch[dma_chan4].transfer_count = pixels_in_scanline * 1;
	//dma_hw->ch[2].transfer_count = pixels_in_scanline * 1;
	//dma_hw->ch[3].transfer_count = pixels_in_scanline * 1;

	//Because of the LCD's orientation, we actually need to scan the image out upside down and flipped
	//base points to the bottom right of the original image
	uint32_t base = pixels_in_scanline + 48 - 52 + (pixels_in_scanline * scanlines_in_active_area) - pixels_in_scanline * 51;


	lcd_den_set(0, true);


	//start vsync pulse

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_sync_lines; wait_line ++) {
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		/*for(uint32_t pixel = 0; pixel < lcd_hblank_back_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		for(uint32_t pixel = 0; pixel < lcd_send_width; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		for(uint32_t pixel = 0; pixel < lcd_hblank_front_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}*/

	}
	//end vsync pulse

	//start vsync back porch

	
	for(uint32_t wait_line = 0; wait_line < lcd_vblank_back_lines; wait_line ++) {
		
		for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		/*for(uint32_t pixel = 0; pixel < lcd_hblank_back_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}
		for(uint32_t pixel = 0; pixel < lcd_send_width; pixel +=4) {
			send_4blank_lcd_pixel();
		}
		for(uint32_t pixel = 0; pixel < lcd_hblank_front_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}*/

	}
	//end vsync back porch


	//frame header
//do some blank header lines....

	static uint16_t line_diff = lcd_active_lines - gg_pixel_height;
	uint16_t header_count = line_diff * 0.5;
	uint16_t footer_count = line_diff - header_count;

	uint32_t y_base;
	uint32_t y_base_offset = (gg_pixel_x_offset + (pixels_in_scanline - gg_pixel_width) * 0.5);
	uint32_t y_target;

	//footer goes at the top when the frame is upside-down


	//frame footer
//do some blank footer lines....
	//for (uint32_t y = gg_pixel_height; y < gg_pixel_height + footer_count; y++)
/*	for (uint32_t y = 0; y < footer_count ; y++)
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{
			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait +=4) {
				send_4blank_lcd_pixel();
			}

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait +=4) {
				send_4blank_lcd_pixel();
			}


			y_base = y * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			y_base += y_base_offset;

			lcd_den_set(1, false);

			//blank line

			for(uint32_t x = y_base; x < y_base + lcd_send_width; x +=4) {
				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;
				
				//send_lcd_pixel(curr_framebuffer[x], hsync, vsync);
				//send_lcd_pixel(curr_framebuffer[x+1], hsync, vsync);
				//send_lcd_pixel(curr_framebuffer[x+2], hsync, vsync);
				//send_lcd_pixel(curr_framebuffer[x+3], hsync, vsync);

				
				send_lcd_pixel(spi_buffer[0]);
				send_lcd_pixel(spi_buffer[1]);
				send_lcd_pixel(spi_buffer[2]);
				send_lcd_pixel(spi_buffer[3]);
			}
			
			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait ++) {				
				send_4blank_lcd_pixel();
			}
			
		}
	}
*/




	//frame body
	//start frame data
	for (uint32_t y = 0; y < lcd_active_lines; y++)
	//for (uint32_t y = lcd_active_lines; y > 0; y--)
	//for (uint32_t y = 0; y < gg_pixel_height + footer_count; y++)	
	//for (int32_t y = gg_pixel_height + footer_count-1  ; y >= 0 ; y--)	
	//for (int32_t y = gg_pixel_height-1; y > 0 ; y--)	
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{
			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait +=4) {
				send_4blank_lcd_pixel();
			}

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait +=4) {
				send_4blank_lcd_pixel();
			}

			//Where in the framebuffer the current scanline starts
			//uint32_t y_base = base - ((y * 310) >> 9) * pixels_in_scanline + 256;

			y_base = (y-1) * pixels_in_scanline;			
			//y_base += (pixels_in_scanline - gg_pixel_width) * 0.5;
			//y_base += (pixels_in_scanline - lcd_send_width)*0.5;
			y_base += y_base_offset;

			//Tell LCD we're starting the active portion of the scanline
			lcd_den_set(1, false);

			///unpack transforms 0b0000rrrrggggbbbb word into 0bbbb000000gggg000000rrrr word
			//Channel order is changed to bgr because that's what the LCD expects
			//Channel bits are spread out so we can do SIMD-within-a-word
			//The interpolator will do linear interpolation on all three channels at once, using a single 32-bit word
			//uint32_t framebuffer_pix_value = curr_framebuffer[y_base] & 4095;
			//uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);
			
			for (uint32_t x = y_base; x < y_base + lcd_send_width; x +=4)
			//for (uint32_t x = y_base + lcd_send_width; x > y_base; x -=4)
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
				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;

				//while(pio_sm_is_tx_fifo_full(lcd_send_pio, lcd_send_sm)) ;

				//This way, we only have to poll on the FIFO status once and then can just blast the three words at once
				//Seems to be significantly faster than pio_sm_put_blocking on each word
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);
				//pix_value >>= 10;
				//pio_sm_put(lcd_send_pio, lcd_send_sm, pix_value & 15);


				//fill the fifo - does not correlate to lcd_pio_hscale. the fifo is just 4 words max

				/*send_lcd_pixel(curr_framebuffer[x]);
				send_lcd_pixel(curr_framebuffer[x-1]);
				send_lcd_pixel(curr_framebuffer[x-2]);
				send_lcd_pixel(curr_framebuffer[x-3]);*/
				send_lcd_pixel(curr_framebuffer[x]);
				send_lcd_pixel(curr_framebuffer[x+1]);
				send_lcd_pixel(curr_framebuffer[x+2]);
				send_lcd_pixel(curr_framebuffer[x+3]);

	
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait +=4) {
				send_4blank_lcd_pixel();
			}			
		}

	}

	//as the frame is output upside-down, the header goes at the bottom
/*	for (uint32_t y = 0; y <= header_count ; y++)
	{
		for (uint32_t w = 0; w < lcd_vscale_factor; w++)
		{
			for(uint32_t wait = 0; wait < lcd_hblank_sync_len; wait +=4) {	
				send_4blank_lcd_pixel();
			}

			for(uint32_t wait = 0; wait < lcd_hblank_back_len; wait +=4) {		
				send_4blank_lcd_pixel();
			}
			
			//repeat the bottom lines at the top
			y_base = (gg_pixel_height + y) * pixels_in_scanline;			
			y_base += y_base_offset;
			y_target = y_base + lcd_send_width;
			
			lcd_den_set(1, false);
			
			//for (uint32_t x = y_base; x < y_target; x +=4) {			
			for (uint32_t x = y_target; x > y_base; x -=4) {	

				while(!pio_sm_is_tx_fifo_empty(lcd_send_pio, lcd_send_sm)) ;
				//send_4blank_lcd_pixel();
				//send_4header_lcd_pixel(spi_buffer[0]);
				send_lcd_pixel(spi_buffer[0]);
				send_lcd_pixel(spi_buffer[1]);
				send_lcd_pixel(spi_buffer[2]);
				send_lcd_pixel(spi_buffer[3]);
			}

			//Signal end of active scanline portion
			lcd_den_set(0, false);
			
			//Send empty pixels during Hblank
			for(uint32_t wait = 0; wait < lcd_hblank_front_len; wait +=4) {				
				send_4blank_lcd_pixel();
			}

		}
	}
*/

	//Start of vblank
	//lcd_den_set(1, false);

	//end frame data

	//start vsync front porch

	//Start of vblank 
	//lcd_den_set(0, false);
	

	//Send empty data for the entirety of vblank
	 
	for(uint32_t pixel = 0; pixel < lcd_hblank_sync_len; pixel +=4) {
		send_4blank_lcd_pixel();
	}

	for(uint32_t wait_line = 0; wait_line < lcd_vblank_front_lines; wait_line ++) {

		for(uint32_t pixel = 0; pixel < lcd_hblank_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		/*for(uint32_t pixel = 0; pixel < lcd_hblank_back_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}

		for(uint32_t pixel = 0; pixel < lcd_send_width; pixel +=4) {
			send_4blank_lcd_pixel();
		}


		for(uint32_t pixel = 0; pixel < lcd_hblank_front_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}*/

		/*for(uint32_t pixel = 0; pixel < lcd_hblank_total_len; pixel +=4) {
			send_4blank_lcd_pixel();
		}*/

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



void config_pios() {
	
	pio_clear_instruction_memory(pio0);
	pio_clear_instruction_memory(pio1);

	//PIO0 captures data from the GG's video bus
	//pio_sm_claim(gg_capture_pio, gg_capture_vblank_sm);
	pio_sm_claim(gg_capture_pio, gg_capture_hblank_sm);
	pio_sm_claim(gg_capture_pio, gg_capture_getdata_sm);


	//PIO1 is used to send data to the LCD without hogging the CPU
	pio_sm_claim(lcd_send_pio, lcd_send_sm);
	//lcd_send_sm = pio_claim_unused_sm(lcd_send_pio, true);

	//pio1 sm1 used for lcd clk
	pio_sm_claim(lcd_send_clk_pio, lcd_send_clk_sm);



	//uint8_t lcd_send_1x = pio_add_program(lcd_send_pio, &lcd_send_program);
	//pio_sm_config lcd_send_1x_config = lcd_send_program_get_default_config(lcd_send_1x);
/*
	uint8_t lcd_send_2x = pio_add_program(lcd_send_pio, &lcd_send_2x_program);
	pio_sm_config lcd_send_2x_config = lcd_send_2x_program_get_default_config(lcd_send_2x);	

	uint8_t lcd_send_4x = pio_add_program(lcd_send_pio, &lcd_send_4x_program);
	pio_sm_config lcd_send_4x_config = lcd_send_4x_program_get_default_config(lcd_send_4x);
*/

	uint8_t lcd_send_spi = pio_add_program(lcd_send_pio, &lcd_send_spi_program);
	pio_sm_config lcd_send_spi_config = lcd_send_spi_program_get_default_config(lcd_send_spi);

	uint8_t lcd_send_clk = pio_add_program(lcd_send_pio, &lcd_send_clk_program);
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
	sm_config_set_sideset_pins(&lcd_send_config, lcd_spi_latch);
	//sm_config_set_sideset_pins(&lcd_send_config, lcd_spi_clk);
	//sm_config_set_set_pins(&lcd_send_config, lcd_spi_latch, 1);
	//sm_config_set_set_pins(&lcd_send_config, lcd_clk, 1);

	sm_config_set_out_shift(&lcd_send_config, false, true, 12);

	sm_config_set_clkdiv(&lcd_send_clk_config, 1);
	sm_config_set_set_pins(&lcd_send_clk_config, lcd_clk, 1);


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
	//pio_sm_set_pindirs_with_mask(lcd_send_pio, lcd_send_sm, (1 << lcd_clk), (1 << lcd_clk) );

	pio_sm_set_pindirs_with_mask(lcd_send_clk_pio, lcd_send_clk_sm, (1 << lcd_clk), (1 << lcd_clk) );

	pio_sm_init(lcd_send_pio, lcd_send_sm, lcd_send, &lcd_send_config);
	pio_sm_set_enabled(lcd_send_pio, lcd_send_sm, true);

	pio_sm_init(lcd_send_clk_pio, lcd_send_clk_sm, lcd_send_clk, &lcd_send_clk_config);
	pio_sm_set_enabled(lcd_send_clk_pio, lcd_send_clk_sm, true);

	
/*
	uint8_t detect_vblank = pio_add_program(gg_capture_pio, &detect_vblank_program);
	pio_sm_config detect_vblank_config = detect_vblank_program_get_default_config(detect_vblank);
	sm_config_set_clkdiv(&detect_vblank_config, 1);
*/

	uint8_t detect_hblank = pio_add_program(gg_capture_pio, &detect_hblank_program);
	pio_sm_config detect_hblank_config = detect_hblank_program_get_default_config(detect_hblank);
	sm_config_set_clkdiv(&detect_hblank_config, 1);

	//uint8_t front_h_porch = pio_add_program(gg_capture_pio, &front_h_porch_program);
	//pio_sm_config front_h_porch_config = front_h_porch_program_get_default_config(front_h_porch);
	//sm_config_set_clkdiv(&front_h_porch_config, 1);


	uint8_t get_data = pio_add_program(gg_capture_pio, &get_data_program);
	pio_sm_config get_data_config = get_data_program_get_default_config(get_data);
	sm_config_set_clkdiv(&get_data_config, 1);
	sm_config_set_in_pins(&get_data_config, gg_D1_pin);
	sm_config_set_in_shift(&get_data_config, false, true, 12);
	//sm_config_set_jmp_pin(&get_data_config, gg_clk_pin);


	//pio_sm_init(gg_capture_pio, gg_capture_vblank_sm, detect_vblank, &detect_vblank_config);
	pio_sm_init(gg_capture_pio, gg_capture_hblank_sm, detect_hblank, &detect_hblank_config);
	pio_sm_init(gg_capture_pio, gg_capture_getdata_sm, get_data, &get_data_config);

	//pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, scanlines_in_active_area + scanlines_to_skip - 1);
	pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, scanlines_in_active_area - 1);
	//pio_sm_put_blocking(gg_capture_pio, gg_capture_hblank_sm, 140);
	pio_sm_exec(gg_capture_pio, gg_capture_hblank_sm, pio_encode_pull(false, true));

	//pio_sm_put_blocking(gg_capture_pio, gg_capture_front_h_porch_sm, h_pixels_to_skip - 1);
	//pio_sm_exec(gg_capture_pio, gg_capture_front_h_porch_sm, pio_encode_pull(false, true));

	pio_sm_put_blocking(gg_capture_pio, gg_capture_getdata_sm, pixels_in_scanline - 1); 
	//pio_sm_put_blocking(gg_capture_pio, gg_capture_getdata_sm, pixels_in_scanline); 
	//pio_sm_put_blocking(gg_capture_pio, gg_capture_getdata_sm, 160); 
	pio_sm_exec(gg_capture_pio, gg_capture_getdata_sm, pio_encode_pull(false, true));

	pio_enable_sm_mask_in_sync(gg_capture_pio, 0b1111);
	
	
}

uint32_t dummy;

void config_dma() {
	for(uint32_t c = 0; c < 12; c++) {
		dma_channel_cleanup(c);
    	dma_channel_unclaim(c);
	}

	//DVI code uses claim_unused_channel, so we must as well, instead of explicitly picking DMA channels
	dma_chan_fb1_write = dma_claim_unused_channel(true);
	dma_chan_fb1_reset = dma_claim_unused_channel(true);
	//dma_chan2 = dma_claim_unused_channel(true);
	//dma_chan3 = dma_claim_unused_channel(true);
	dma_chan_fb2_write = dma_claim_unused_channel(true);
	dma_chan_fb2_reset = dma_claim_unused_channel(true);


	//PIO0 SM2 is the one that actually sends out pixel data
/*
	dma_channel_config g = dma_channel_get_default_config(dma_chan2);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, dma_chan0);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(gg_capture_pio, gg_capture_getdata_sm, false));
	dma_channel_configure(
			dma_chan2,
			&g,
			&dummy,
			&gg_capture_pio->rxf[gg_capture_getdata_sm],
			//The first few active display scanlines are actually blank
			//We skip them by taking data from the PIO and discarding it into the dummy variable
			//We than chain to dma_chan0 
			pixels_to_skip,
			false);
*/

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
			//Then chain to dma_chan1
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan0's write address register to point to the start of framebuffer0
	//Then chain to dma_chan3
	dma_channel_config e = dma_channel_get_default_config(dma_chan_fb1_reset);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	//channel_config_set_chain_to(&e, dma_chan3);
	channel_config_set_chain_to(&e, dma_chan_fb2_write);
	//channel_config_set_chain_to(&e, dma_chan0);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan_fb1_reset,
			&e,
			&(dma_hw->ch[dma_chan_fb1_write].write_addr),
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
	channel_config_set_dreq(&g, pio_get_dreq(gg_capture_pio, gg_capture_getdata_sm, false));
	dma_channel_configure(
			dma_chan3,
			&g,
			&dummy,
			&gg_capture_pio->rxf[gg_capture_getdata_sm],
			pixels_to_skip,
			false);
*/


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
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan4's write address and chain into dma_chan2 to restart the whole DMA chain
	e = dma_channel_get_default_config(dma_chan_fb2_reset);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	//channel_config_set_chain_to(&e, dma_chan2);
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

//uint16_t audio_bias_midpoint = 2048; //ADC is 0-4095 2048 is 1.65v midpoint of 3.3v
//uint16_t audio_bias_midpoint = 2829; //2.28v
//uint16_t audio_bias_midpoint = 1900; //1.53v
static const int16_t audio_bias_midpoint = 1948; //1.57v
//uint16_t audio_bias_midpoint = 1775; //1.43v
//uint16_t audio_bias_midpoint = 1737; //1.40v
//uint16_t audio_bias_midpoint = 1514; //1.22v
//uint16_t audio_bias_midpoint = 1340; //1.08v
//uint16_t audio_bias_midpoint = 1290; //1.04v
//uint16_t audio_bias_midpoint = 1240; //1.0v

//uint16_t audio_bias_midpoint = 1365; //1365 is midpoint of 1.1v
const uint16_t audio_volume_multiplier = 1;


/*
static int32_t hp_lowpass_int = 0;
static const int32_t alpha_int = (int32_t)(0.996f * 65536.0f);

int32_t process_highpass_int(int32_t input)
{
	int32_t diff = input - hp_lowpass_int;
	hp_lowpass_int += (alpha_int * diff) >> 16;
	return input - (hp_lowpass_int >> 16);
}
*/

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


bool __not_in_flash_func(adc_timer_callback)(struct repeating_timer *t)
{
	static uint channel = 0; //0 = left, 1 = right

	//static int32_t sum_l = audio_bias_midpoint;
	//static int32_t sum_r = audio_bias_midpoint; 
	static int32_t sum_l = 2047;
	static int32_t sum_r = 2047;
	sum_l = 2047;
	sum_r = 2047;
	//accum_l = 0;
	
	#define AVG 1
	//for (int i = 0; i < AVG; i++) 
	{
		
		adc_select_input(gg_audio_l_pin - ADC_BASE_PIN);
		//sum += ((adc_read() >> 3) - 512);
		//sum_l += (adc_read() - audio_bias_midpoint) / AVG;		
		//sum_l += process_slow_iir_dc_block_l(adc_read());
		

		adc_select_input(gg_audio_r_pin - ADC_BASE_PIN);
		//sum_r += (adc_read() - audio_bias_midpoint) / AVG;	
		//sum_r += process_slow_iir_dc_block_r(adc_read());

//#ifdef DEBUG			
		//printf("ADC Timer Sample: %.2f\n", sum);
		//printf("ADC Timer Sample: %i\n", sum);
//#endif
		//adc_select_input(gg_audio_r_pin - ADC_BASE_PIN);
		//sum -= adc_read();
	}
	
	//int16_t sample = (int16_t)(sum - 2048);
	//uint16_t raw = sum / AVG;

//	int16_t sample = (int16_t)(adc_read() - 2048);
	//int16_t sample = (int16_t)(raw - 2048);	
	//int16_t sample = (int16_t)sum_l;
	//sample = sample < -1024 ? 0 : sample; //prevent noisey whine when no input, for testing at least...
	//sample = process_highpass_int(sample);
	//accum_l += sample;
	accum_l += (int16_t)sum_l;

	//sample = (int16_t)sum_r;
	//sample = sample < -1024 ? 0 : sample; //prevent noisey whine when no input, for testing at least...
	//sample = process_highpass_int(sample);
	//accum_r += sample;
	accum_r += (int16_t)sum_r;


	//if(++decimate >= ADC_SAMPLE_MULTIPLIER)
	//{

		//find the bias midpoint in the first 100 miliseconds of running
		/*if(time_us_32() - startup_time < 100000)
		{
			//audio_bias_midpoint = sample / ADC_SAMPLE_MULTIPLIER;
			audio_bias_midpoint += ((sample / ADC_SAMPLE_MULTIPLIER) - audio_bias_midpoint) * 0.1;
		}*/
		

		//decimate = 0;

		//accum_l = accum_l >> 0;
		//accum_l = accum_l / ADC_SAMPLE_MULTIPLIER;
		//accum_r = accum_r / ADC_SAMPLE_MULTIPLIER;

		//int16_t current_l = prev_l + ((accum_l - prev_l) >> 4);

		//audio_l_buffer[audio_write_pos] = (current_l);// >> 4;
		//audio_l_buffer[audio_write_pos] = (current_l * 4);// >> 4;


		//audio_l_buffer[audio_write_pos] = (accum_l >> 3);
		//audio_l_buffer[audio_write_pos] = (int16_t)accum_l * audio_volume_multiplier;
		//audio_r_buffer[audio_write_pos] = (int16_t)accum_r * audio_volume_multiplier;
		audio_buffer[audio_write_pos] = (int16_t)accum_l;
		audio_write_pos = (audio_write_pos + 1) % AUDIO_BUFFER_SIZE;
		audio_buffer[audio_write_pos] = (int16_t)accum_r;
		audio_write_pos = (audio_write_pos + 2) % AUDIO_BUFFER_SIZE;

		//printf("ADC Timer Sample: %i %i\n", (int16_t)accum_l, audio_l_buffer[audio_write_pos]);

		//printf("ADC Timer Sample: %i\n", audio_l_buffer[audio_write_pos]);

		//audio_l_buffer[audio_write_pos] = (accum_l);
		//audio_l_buffer[audio_write_pos] = (accum_l * 4);// >> 4;
		//audio_l_buffer[audio_write_pos] = prev + (((accum_l * 90) - prev) >> 4);
		//prev_l = current_l;
		//prev = accum_l;
		accum_l = 0;
		accum_r = 0;
		
		//audio_l_buffer[audio_write_pos] = (sample * 15);
		//audio_l_buffer[audio_write_pos] = sine[audio_write_pos % SINE_SIZE] / 2;

		//channel = (channel + 1) % 2; //switch channel for next sample
	//}

#ifdef DEBUG
		//printf("ADC Timer Sample: %.2f\n", sample);
#endif

	return true;
}



bool __not_in_flash_func(sampler_timer_callback)(struct repeating_timer *t)
{
	adc_select_input(gg_audio_l_pin - ADC_BASE_PIN);
	audio_buffer[audio_write_pos] = adc_read();
	audio_write_pos = (audio_write_pos + 1) % AUDIO_BUFFER_SIZE;

	adc_select_input(gg_audio_r_pin - ADC_BASE_PIN);
	//audio_r_buffer[audio_write_pos] = adc_read();
	audio_buffer[audio_write_pos] = adc_read();
	audio_write_pos = (audio_write_pos + 1) % AUDIO_BUFFER_SIZE;
	
	return true;
}

//static uint16_t sample_count = 0;

bool __not_in_flash_func(dvi_audio_timer_callback)(struct repeating_timer *t)
{
	//while(true)
	{
		//get how many audio samples for the dvi buffer
		int size = get_write_size(&dvi0.audio_ring, true);
		if(size == 0) return true;
		if(size >= AUDIO_BUFFER_SIZE)
		{
			size = AUDIO_BUFFER_SIZE;
		}

		//get where we need to write the audio sample to
		audio_sample_t *audio_ptr = get_write_pointer(&dvi0.audio_ring);
		//audio_sample_t *audio_buffer = get_buffer_top(&dvi0.audio_ring);
		uint32_t audio_offset = get_write_offset(&dvi0.audio_ring);
		audio_sample_t sample;

		for(int cnt = 0; cnt < size; cnt++)
		{

			//while(audio_write_pos == audio_read_pos) {};
			if(audio_write_pos == audio_read_pos)
			{
				//break;
				 sample.channels[0] = 0; //silence on underrun?
				 sample.channels[1] = 0;
			}
			else 
			{

				//sample.channels[0] = 0;
				//sample.channels[1] = 0;

				sample.channels[0] = (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint;
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//sample.channels[0] = audio_r_buffer[audio_read_pos];
				sample.channels[1] = (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint;
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);
				//sample.channels[1] = audio_r_buffer[audio_read_pos];
				

				//sample.channels[0] = audio_l_buffer[audio_read_pos];
				//sample.channels[1] = audio_l_buffer[audio_read_pos];

				//sample.channels[0] = audio_r_buffer[audio_read_pos];
				//sample.channels[1] = audio_r_buffer[audio_read_pos];
				//sample.channels[1] = 0;

				//sample.channels[1] = audio_r_buffer[audio_read_pos];				
				//audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//audio_buffer[audio_offset] = sample;
				audio_offset = (audio_offset + 1) % (DVI_AUDIO_BUFFER_SIZE - 1);
			}

			*audio_ptr++ = sample;
			increase_write_pointer(&dvi0.audio_ring, 1);
			audio_ptr = get_write_pointer(&dvi0.audio_ring);
			//sample_count = (sample_count +1) % SINE_SIZE;
			//sample_count++;
			
		}

		//set_write_offset(&dvi0.audio_ring, audio_offset);
		//increase_write_pointer(&dvi0.audio_ring, sample_count);
		//increase_write_pointer(&dvi0.audio_ring, size);

	}
	
    return true;
}

int adc_dma_chan_sample = -1;
int adc_dma_chan_control = -1;

bool __not_in_flash_func(dvi_audio_timer_callback_dma)(struct repeating_timer *t)
{
	
//printf("DMA channel claimed: %d\n", adc_dma_chan);

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
		//audio_sample_t *audio_buffer = get_buffer_top(&dvi0.audio_ring);
		uint32_t audio_offset = get_write_offset(&dvi0.audio_ring);
		audio_sample_t sample;

		//Should be using trans_count as write_addr is unreliable
		uint32_t current_trans_count = dma_hw->ch[adc_dma_chan_sample].transfer_count;
		uint32_t samples_written = AUDIO_BUFFER_SIZE - current_trans_count;
		//uint32_t current_dma_addr = dma_hw->ch[adc_dma_chan].write_addr;
		//uint32_t samples_written = (current_dma_addr - (uint32_t)audio_buffer) / 2;

		audio_write_pos = samples_written - (samples_written % 2);
		
		//printf("Trans Count: %u | Buffer Size: %u | samples requested: %u | Samples Available: %d | write_pos: %u | read_pos: %u \n",
         //  current_trans_count, AUDIO_BUFFER_SIZE, size, audio_write_pos-audio_read_pos, audio_write_pos, audio_read_pos);
		//printf("Trans Count: %u | Buffer Size: %u | samples requested: %u | Samples written: %u | write_pos: %u | read_pos: %u | FIFO level: %d\n",
        //   current_trans_count, AUDIO_BUFFER_SIZE, size, samples_written, audio_write_pos, audio_read_pos, adc_fifo_get_level());
		//printf("DMA addr: 0x%08X | Samples written: %u | write_pos: %u | read_pos: %u | FIFO level: %d | Request Size: %d | Buffer Size: %d\n",
        //   current_dma_addr, samples_written, audio_write_pos, audio_read_pos, adc_fifo_get_level(), size, AUDIO_BUFFER_SIZE);
		
		for(int cnt = 0; cnt < size; cnt++)
		{
			//while(audio_write_pos == audio_read_pos) {};
			if(audio_write_pos == audio_read_pos)
			{
				//break;
				 sample.channels[0] = 0; //silence on underrun?
				 sample.channels[1] = 0;
			}
			else 
			{				
				//sample.channels[0] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ;
				sample.channels[0] =  (int16_t)(audio_buffer[audio_read_pos]) - 2048 ;
				//sample.channels[0] =  0;
				//sample.channels[0] = process_slow_iir_dc_block_l( (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint );
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);

				//sample.channels[1] =  (int16_t)(audio_buffer[audio_read_pos]) - audio_bias_midpoint ;
				sample.channels[1] =  (int16_t)(audio_buffer[audio_read_pos]) - 2048 ;
				//sample.channels[1] = 0; 
				//sample.channels[1] = process_slow_iir_dc_block_r( (int16_t)audio_buffer[audio_read_pos] - audio_bias_midpoint );
				audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);
		
				//audio_read_pos = ((audio_read_pos + 1) % AUDIO_BUFFER_SIZE);
				audio_offset = (audio_offset + 1) % (DVI_AUDIO_BUFFER_SIZE - 1);

				//if (cnt % 40 == 0) {
					//printf("Read pos: %u | Sample L: %d\n", audio_read_pos, sample.channels[0]);
					//printf("Read pos: %u | Sample L: %d R: %d\n", audio_read_pos, sample.channels[0], sample.channels[1]);
				//	printf("Read pos: %u \n", audio_buffer[audio_read_pos]);
				//}

			}

			*audio_ptr++ = sample;
			increase_write_pointer(&dvi0.audio_ring, 1);
			audio_ptr = get_write_pointer(&dvi0.audio_ring);
			//sample_count = (sample_count +1) % SINE_SIZE;
			//sample_count++;

			
			//if (cnt % 100 == 0) {
			//	printf("Read pos: %u | Sample L: %u R: %u\n", audio_read_pos, sample.channels[0], sample.channels[1]);
				//printf("Read pos:\n");
			//}

		}

		//set_write_offset(&dvi0.audio_ring, audio_offset);
		//increase_write_pointer(&dvi0.audio_ring, sample_count);
		//increase_write_pointer(&dvi0.audio_ring, size);

	}
	
    return true;
}

void __isr audio_dma_irq0_handler(void)
{
	dma_hw->ints0 = 1u << adc_dma_chan_sample;  //clear interrupt
	//audio_write_pos = 
	dma_channel_set_trans_count(adc_dma_chan_sample, AUDIO_BUFFER_SIZE, true); //restart dma

}

void __isr audio_dma_irq1_handler(void)
{
	dma_hw->ints1 = 1u << adc_dma_chan_sample;  //clear interrupt
	//audio_write_pos = 
	dma_channel_set_trans_count(adc_dma_chan_sample, AUDIO_BUFFER_SIZE, true); //restart dma

}

void __not_in_flash_func(config_audio)()
{

//	gpio_init(gg_audio_l_pin);
//	gpio_disable_pulls(gg_audio_l_pin);
//	adc_gpio_init(gg_audio_l_pin);
	

	/*gpio_init(gg_audio_pwr_pin);
	gpio_set_dir(gg_audio_pwr_pin, GPIO_OUT);
	gpio_put(gg_audio_pwr_pin, 1);*/

	for(uint32_t i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audio_buffer[i] = 2048;
		//audio_r_buffer[i] = 0;
	}

	adc_init();
	adc_gpio_init(gg_audio_l_pin); //enable adc and disabled gpio on these pins
	adc_gpio_init(gg_audio_r_pin);
	
	//adc_select_input(gg_audio_r_pin - ADC_BASE_PIN);
	//adc_select_input(gg_audio_l_pin - ADC_BASE_PIN);
	adc_set_temp_sensor_enabled(false);

	adc_set_round_robin(0b0011); //sample adc pins 1 and 2 i.e. 26/27

	float clk_div = (48 * 1000 * 1000) / (AUDIO_SAMPLE_RATE * AUDIO_CHANNEL_COUNT);
	adc_set_clkdiv(clk_div - 1.0f);
		
	adc_fifo_setup(true, true, 1, false, false);


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

	/*if(AUDIO_DMA_IRQ == DMA_IRQ_0)
	{
		dma_channel_set_irq0_enabled(adc_dma_chan_sample, true);
		irq_set_exclusive_handler(AUDIO_DMA_IRQ, audio_dma_irq0_handler);
	}
	else
	{
		dma_channel_set_irq1_enabled(adc_dma_chan_sample, true);
		irq_set_exclusive_handler(AUDIO_DMA_IRQ, audio_dma_irq1_handler);
	}
	
	irq_set_enabled(AUDIO_DMA_IRQ, true);*/

	//dma_channel_configure(adc_dma_chan, &adc_dma_config, audio_l_buffer, &adc_hw->fifo, AUDIO_BUFFER_SIZE, true);
	//dma_channel_configure(adc_dma_chan, &adc_dma_config, audio_l_buffer, &adc_hw->fifo, 0xFFFFFFFF, true);
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


	//dma_channel_start(adc_dma_chan_sample);
	dma_channel_start(adc_dma_chan_control);

	adc_run(true);
	
	//timer for sending dvi audio
	//add_repeating_timer_ms(2, dvi_audio_timer_callback, NULL, &dvi_audio_timer);
	add_repeating_timer_ms(1, dvi_audio_timer_callback_dma, NULL, &dvi_audio_timer);

	//timer for adc sampling
	//add_repeating_timer_us(1000000 / (AUDIO_SAMPLE_RATE * ADC_SAMPLE_MULTIPLIER), adc_timer_callback, NULL, &adc_timer);
	//add_repeating_timer_us(1000000 / (AUDIO_SAMPLE_RATE * ADC_SAMPLE_MULTIPLIER), sampler_timer_callback, NULL, &adc_timer);	

}



// void config_backlight_supply(uint8_t pwm_backlight_slice) {
// 	//adc_init();
// 	adc_gpio_init(backlight_fdbck);
// 	adc_select_input(backlight_fdbck - ADC_BASE_PIN);
// 	//adc_run(true);
// 	//adc_fifo_setup(true, false, 0, 0, 0);

// 	pwm_hw->slice[pwm_backlight_slice].cc = 250;
// 	sleep_ms(50);

// 	float tensao_media = 0;

// 	/*while(tensao_media < 19 || tensao_media > 20) {
// 		for(uint32_t m = 0; m < 5; m++) {
// 			sleep_ms(2);
// 			tensao_media += adc_fifo_get();
// 		}

// 		tensao_media *= 3.3;
// 		tensao_media *= 11;
// 		tensao_media /= 5;
// 		tensao_media /= 4095;

// 		if(tensao_media < 10) {
// 			pwm_hw->slice[pwm_backlight_slice].cc += 50;
// 		}
// 		else if(tensao_media < 15) {
// 			pwm_hw->slice[pwm_backlight_slice].cc += 20;
// 		}
// 		else if(tensao_media < 19) {
// 			pwm_hw->slice[pwm_backlight_slice].cc += 10;
// 		}
// 		else if(tensao_media > 21) {
// 			pwm_hw->slice[pwm_backlight_slice].cc -= 20;
// 		}
// 		else if(tensao_media > 19) {
// 			pwm_hw->slice[pwm_backlight_slice].cc -= 10;
// 		}

// 		gpio_put(led_pin, !gpio_get(led_pin));
// 	}*/
// 	pwm_hw->slice[pwm_backlight_slice].cc = 500;
// 	//pwm_hw->slice[pwm_backlight_slice].cc = 20;
// }


// uint8_t config_backlight_pwm() {
// 	gpio_set_function(lcd_backlight, GPIO_FUNC_PWM);
// 	pwm_backlight_slice = pwm_gpio_to_slice_num(lcd_backlight);

// 	pwm_config config = pwm_get_default_config();
// 	pwm_config_set_phase_correct(&config, false);	//default is false anyway
// 	pwm_config_set_clkdiv_int(&config, 1);
// 	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	
// 	//uint16_t pwm_wrap_target = ((DVI_TIMING.bit_clk_khz * 1000) / 30000);
// 	pwm_config_set_wrap(&config, pwm_wrap_target);
// 	//pwm_config_set_wrap(&config, (DVI_TIMING.bit_clk_khz * 1000) / 30000);

// 	pwm_init(pwm_backlight_slice, &config, true);

// 	return pwm_backlight_slice;
// }
	

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

void draw_rectangle_empty(uint16_t * current_framebuffer, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint32_t ypos = (y * pixels_in_scanline) + x;
	uint32_t ypos2 = ((y + h) * pixels_in_scanline) + x;
	uint32_t xpos_base = (y * pixels_in_scanline) + x;
	uint32_t xpos2_base = (y * pixels_in_scanline) + x + w;
	
	for(uint32_t line = 0; line < w;line++)
	{
		//top line
		current_framebuffer[ypos + line] = color;
		
		//bottom line
		current_framebuffer[ypos2 + line] = color;
	}

	for(uint32_t line = 0; line < h; line++)
	{
		//left line
		current_framebuffer[xpos_base + (line * pixels_in_scanline)] = color;

		//right line 		
		current_framebuffer[xpos2_base + (line * pixels_in_scanline)] = color;
	}

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

	draw_rectangle_empty(framebuffer, gg_pixel_x_offset + 60, 0, gg_pixel_width, gg_pixel_height, 0xFFF);
	draw_rectangle_empty(framebuffer2, gg_pixel_x_offset + 60, 0, gg_pixel_width, gg_pixel_height, 0xFFF);

}


void fill_framebuffer_with_test_pattern2() {
	uint16_t test_divs = pixels_in_scanline / 8;	
	uint16_t row_size = scanlines_in_active_area / 5;
	uint16_t row_diff = 3;
	for(uint32_t y = 0; y < scanlines_in_active_area; y++) {
		uint16_t row_val = (y / row_size);
		row_val = row_diff * row_val;
		row_val = 15 - row_val;

		//uint16_t row_val = 15;

		for(uint32_t x = 0; x < pixels_in_scanline; x++) {
			uint16_t pixel = 0x800;
			//uint16_t pixel = 0xFFF;

			framebuffer[x + (y * pixels_in_scanline)]  = pixel;
			framebuffer2[x + (y * pixels_in_scanline)]  = pixel;

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
			if(!dma_channel_is_busy(dma_chan_fb1_write)) 
			{
				curr_framebuffer = framebuffer2;
			}
			else 
			{
				curr_framebuffer = framebuffer;
			
				
				uint32_t frame_size_to_use = FRAME_SIZE;// * 2;//_MIN; //FRAME_SIZE_HALF;

				//frame_size_to_use -= pixels_in_scanline * 16; //51;
				
				//memcpy(send_buffer, curr_framebuffer, pixels_in_scanline  * 144); // Copy full buffer

				//char buf[64];
				//uint32_t count = tud_cdc_read(buf, sizeof(buf));
				tud_cdc_read_flush();
				//if(count > 0 && buf[0] == 0x0A) //enter received - send the current frame buffer
				{		

					static uint32_t offset = 0;
					//uint32_t CHUNK_SIZE = CFG_TUD_CDC_TX_BUFSIZE;
					static uint32_t chunk = 0;

					while(offset < frame_size_to_use)
					{
						chunk = (frame_size_to_use - offset < CFG_TUD_CDC_TX_BUFSIZE) ? frame_size_to_use - offset : CFG_TUD_CDC_TX_BUFSIZE;


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


void __not_in_flash_func(core1_main)() 
{

	//configure audio events on current core
	config_audio();

	dvi_register_irqs_this_core(&dvi0, DVI_DMA_IRQ);

	dvi_start(&dvi0);
	dvi_scanbuf_main_12bpp_noqueue(&dvi0, framebuffer, framebuffer2, dma_chan_fb1_write, dma_chan_fb2_write);

	__builtin_unreachable();
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
	//spi_read_blocking(in_sr_spi, 0xFF , &data, 1);
	spi_read_blocking(btn_sr_spi, 0 , &data, 1);

	//printf("SPI IN DATA %i\n", data);

	gg_now = !(data & 0x1);
	gg_start_now = !(data & 0x8);
	gg_btn1_now = !(data & 0x4);
	gg_btn2_now = !(data & 0x2);


}


char* title = "PICO GG LCD";
const uint8_t font_size = 8;
const uint16_t font_color = 0xDDD;
const uint16_t overlay_color_base = 0x066F;
const uint8_t overlay_height = 22;
const uint16_t overlay_ypos = gg_pixel_height - overlay_height;
const uint16_t overlay_xpos = gg_pixel_x_offset + (pixels_in_scanline - gg_pixel_width) * 0.5;



void draw_char(uint16_t * current_framebuffer, uint16_t x, uint16_t y, const char c, uint16_t color)
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

void draw_overlay(uint16_t * current_framebuffer, uint8_t fps)
{
	//just draw some lines for testing...
	uint16_t pixel = overlay_color_base;

/*
	if(gg_btn1_now)
	{
		pixel = 0xF00;
	}
	else if (gg_btn2_now)
	{
		pixel = 0x0F0;
	}
	else if (gg_start_now)
	{
		pixel = 0x00F;
	}
*/

	//draw a background
	for(uint32_t y = overlay_ypos; y < overlay_ypos + overlay_height; y++) 
	{
		//uint16_t row_val = 15;

		for(uint32_t x = 0; x < pixels_in_scanline; x++) {
		
			current_framebuffer[x + (y * pixels_in_scanline)]  = pixel;
			//framebuffer2[x + (y * pixels_in_scanline)]  = ~pixel;
			current_framebuffer[x + (y * pixels_in_scanline)]  = pixel;

			//framebuffer[x + (y * pixels_in_scanline)] = pixel;
			//framebuffer2[x + (y * pixels_in_scanline)] = ~pixel;
		}
	}	

	//draw some text
	draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + 2, title, font_color);
	static char fps_str[8];
	sprintf(fps_str, "%i fps", fps);

	draw_string(current_framebuffer, overlay_xpos + font_size*2 + strlen(title) * 8, overlay_ypos + 2, fps_str, font_color);

	if(gg_btn1_now)
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4, "BTN1 Pressed!", font_color);
	}
	if(gg_btn2_now)
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4, "BTN2 Pressed!", font_color);
	}
	if(gg_start_now)
	{
		draw_string(current_framebuffer, overlay_xpos + font_size, overlay_ypos + font_size + 4, "START Pressed!", font_color);
	}

}

uint32_t last_frame_time = 0;
uint32_t last_pwm_feedback_time = 0;
void core0_main() 
{

	gpio_put(lcd_den, 0);
	//gpio_put(lcd_hsync, 0);
	//gpio_put(lcd_vsync, 0);
	gpio_put(led_pin, 1);
	
	gpio_put(lcd_clk, 0);
	//gpio_put(lcd_backlight, 0);
	
	//enable the backlight supply from the booster
	gpio_init(lcd_dim);
	gpio_set_dir(lcd_dim, GPIO_OUT);
	gpio_put(lcd_dim, 1);


	/*adc_init();
	adc_gpio_init(brightness_pot);
	adc_select_input(brightness_pot - ADC_BASE_PIN);
	adc_run(true);
	adc_fifo_setup(true, false, 0, 0, 0);*/

	fill_framebuffer_with_test_pattern();
	//fill_framebuffer_with_test_pattern2();

	//dma_channel_start(dma_chan2);
	dma_channel_start(dma_chan_fb1_write);

	while(1) {

		watchdog_update();

		read_in_spi();

/*
		last_gg = gg_now;
		gg_now = !gpio_get(gg_SMS_pin);

		if(last_gg == gg_now) is_gg = gg_now;

*/


		uint32_t start = time_us_32();

		//update lcd after 15ms for just over 60fps
		//if(start - last_frame_time > 15000)
		if(start - last_frame_time > 10000)
		{
			//while(dma_channel_is_busy(dma_chan_fb1_write) && dma_channel_is_busy(dma_chan_fb2_write)) ;

			if(dma_channel_is_busy(dma_chan_fb1_write)) framebuffer_to_use = framebuffer2;
			else framebuffer_to_use = framebuffer;

			//framebuffer_to_use = framebuffer2;

			//draw_overlay(framebuffer_to_use, 1000000 / (start - last_frame_time));
		
			/*if(gg_btn1_now || gg_btn2_now || gg_start_now)
			{
				draw_overlay(framebuffer, 1000000 / (start - last_frame_time));
				draw_overlay(framebuffer2, 1000000 / (start - last_frame_time));
			}*/

			update_lcd_gg(framebuffer_to_use);
			last_frame_time = start;
#ifdef DEBUG
			uint32_t end = time_us_32();
			
			printf("Rendering time: %i us, time since last: %i us\n", end - start, start - last_frame_time); 
#endif
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

/*
		start = time_us_32();
		if(start - last_pwm_feedback_time > 5000)
		{

			adc_select_input(backlight_fdbck - ADC_BASE_PIN);

			uint16_t v_div = adc_read();
			float v_feedback = v_div * (pwm_feedback_target / 4095.f);
			float error = (pwm_feedback_target * 0.5f) - v_feedback;

			int32_t adjustment = (int32_t)(error * 0.01f * (float)pwm_wrap_target / (pwm_feedback_target));

			pwm_hw->slice[pwm_backlight_slice].cc += adjustment;

			if(pwm_hw->slice[pwm_backlight_slice].cc < pwm_min_duty)
				pwm_hw->slice[pwm_backlight_slice].cc = pwm_min_duty;
			
			if(pwm_hw->slice[pwm_backlight_slice].cc > pwm_max_duty) 
				pwm_hw ->slice[pwm_backlight_slice].cc = pwm_max_duty;


#ifdef DEBUG
			printf("VDiv: %.2f, Feedback: %.2fV, Error: %.2f, Adjustment: %i, Duty: %u\n", v_div,  v_feedback, error, adjustment, pwm_hw->slice[pwm_backlight_slice].cc);
#endif
			last_pwm_feedback_time = time_us_32();
			//sleep_ms(5); 
		}
*/

	}
	__builtin_unreachable();
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

	spi_set_format(btn_sr_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST);
}

/*
void shutdown_before_reset(void) {
    adc_run(false);
    adc_fifo_drain();

    if (adc_dma_chan >= 0) {
        dma_channel_abort(adc_dma_chan);
        dma_channel_set_irq0_enabled(adc_dma_chan, false);
        dma_channel_set_irq1_enabled(adc_dma_chan, false);
    }

    irq_set_enabled(DMA_IRQ_0, false);
    irq_set_enabled(DMA_IRQ_1, false);

    dma_hw->ints0 = 0xFFFFFFFF;
    dma_hw->ints1 = 0xFFFFFFFF;
}

#define BOOTSEL_MAGIC 0xB0075E57
#define BOOTSEL_MAGIC_ADDR ((volatile uint32_t *)0x20000000)

void check_reset_cause(void) {
    // Read the reset cause bits
    uint32_t reset_cause = vreg_and_chip_reset_hw->chip_reset;

    printf("Reset cause register: 0x%08x\n", reset_cause);

	bool had_por, had_run, had_psm_restart, psm_restart_flag = false;
    if (reset_cause & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) {
        printf("→ Power-on reset (or brown-out)\n");
		had_por = true;
    }

    if (reset_cause & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) {
        printf("→ RUN pin reset (or external reset)\n");
		had_run = true;
    }

    if (reset_cause & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) {
        printf("→ Reset From Debug Port\n");
		had_psm_restart = true;
    }

    if (reset_cause & VREG_AND_CHIP_RESET_CHIP_RESET_PSM_RESTART_FLAG_BITS) {
        printf("→ PSM Restart From Debugger\n");
		psm_restart_flag = true;
    }

    // Clear the reset cause bits so they don't persist on next reset
    vreg_and_chip_reset_hw->chip_reset = reset_cause;

	if(!had_por && !had_run)
	{
		if(watchdog_caused_reboot()){
			shutdown_before_reset();
			//rom_reset_usb_boot(0, 0);
			*BOOTSEL_MAGIC_ADDR = BOOTSEL_MAGIC;
			//watchdog_enable(1, false);
			watchdog_reboot(0,0,0);
			while(true){ watchdog_update(); };
		}
	}

}
*/

int __not_in_flash_func(main)() 
{
	
	startup_time = time_us_32();

	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	//vreg_set_voltage(VREG_VOLTAGE_1_10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	stdio_init_all();

	tusb_init(); //initialise TinyUSB stack

	//check if we might have been reset by watchdog - in which case reboot to bootsel mode
	//check_reset_cause();
	//watchdog_enable(3000, false); //restart if things hang


	gpio_init_mask(0b11111111111111111111111111111111);
	gpio_set_dir_out_masked(1 << led_pin);
	gpio_set_dir_out_masked(1 << lcd_rst);
	gpio_set_dir_out_masked(1 << lcd_den);
	//gpio_set_dir_out_masked(1 << lcd_hsync);
	//gpio_set_dir_out_masked(1 << lcd_vsync);
	//gpio_set_dir_out_masked(1 << lcd_backlight);
	
	//adc_init();
	
	//uint8_t pwm_backlight_slice = config_backlight_pwm();
	//config_backlight_supply(pwm_backlight_slice);


	//These functions MUST be called before dvi_init, since they unclaim all DMA channels and clear PIO memory
	init_lcd();
	config_pios();
	config_dma();
	//config_interp();
	config_in_spi();



	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = pico_gg_lcd_conf;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	
	//HDMI AUDIO
	
 	for(uint32_t i = 0; i < DVI_AUDIO_BUFFER_SIZE; i++)
	{
		dvi_audio_buffer[i].channels[0] = 0;
		dvi_audio_buffer[i].channels[1] = 0;
	}		

	dvi_get_blank_settings(&dvi0)->top = 0;
	dvi_get_blank_settings(&dvi0)->bottom = 0;
	dvi_audio_sample_buffer_set(&dvi0, dvi_audio_buffer, DVI_AUDIO_BUFFER_SIZE);
	dvi_set_audio_freq(&dvi0, AUDIO_SAMPLE_RATE, DVI_AUDIO_CTS, 6272);


	multicore_reset_core1();


	multicore_launch_core1(core1_main); 
	
	core0_main();

	return 0;
}
 
