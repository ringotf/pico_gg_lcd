#include <stdlib.h>
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "dvi.h"
#include "dvi_timing.h"
#include "dvi_serialiser.h"
#include "tmds_encode.h"

// Time-critical functions pulled into RAM but each in a unique section to
// allow garbage collection
#define __dvi_func(f) __not_in_flash_func(f)
#define __dvi_func_x(f) __scratch_x(__STRING(f)) f
#define pixels_in_scanline 300
#define scanlines_in_active_area 192
#define header_scanlines ((240 - scanlines_in_active_area) / 1)
#define pixels_border (320 - pixels_in_scanline) / 2

// We require exclusive use of a DMA IRQ line. (you wouldn't want to share
// anyway). It's possible in theory to hook both IRQs and have two DVI outs.
static struct dvi_inst *dma_irq_privdata[2];
static void dvi_dma0_irq();
static void dvi_dma1_irq();

void dvi_init(struct dvi_inst *inst, uint spinlock_tmds_queue, uint spinlock_colour_queue) {
	dvi_timing_state_init(&inst->timing_state);
	dvi_serialiser_init(&inst->ser_cfg);
	for (int i = 0; i < N_TMDS_LANES; ++i) {
		inst->dma_cfg[i].chan_ctrl = dma_claim_unused_channel(true);
		inst->dma_cfg[i].chan_data = dma_claim_unused_channel(true);
		inst->dma_cfg[i].tx_fifo = (void*)&inst->ser_cfg.pio->txf[inst->ser_cfg.sm_tmds[i]];
		inst->dma_cfg[i].dreq = pio_get_dreq(inst->ser_cfg.pio, inst->ser_cfg.sm_tmds[i], true);
	}
	inst->late_scanline_ctr = 0;
	inst->tmds_buf_release_next = NULL;
	inst->tmds_buf_release = NULL;
	queue_init_with_spinlock(&inst->q_tmds_valid,   sizeof(void*),  8, spinlock_tmds_queue);
	queue_init_with_spinlock(&inst->q_tmds_free,    sizeof(void*),  8, spinlock_tmds_queue);
	queue_init_with_spinlock(&inst->q_colour_valid, sizeof(void*),  8, spinlock_colour_queue);
	queue_init_with_spinlock(&inst->q_colour_free,  sizeof(void*),  8, spinlock_colour_queue);

	dvi_setup_scanline_for_vblank(inst->timing, inst->dma_cfg, true, &inst->dma_list_vblank_sync);
	dvi_setup_scanline_for_vblank(inst->timing, inst->dma_cfg, false, &inst->dma_list_vblank_nosync);
	dvi_setup_scanline_for_active(inst->timing, inst->dma_cfg, (void*)SRAM_BASE, &inst->dma_list_active);
	dvi_setup_scanline_for_active(inst->timing, inst->dma_cfg, NULL, &inst->dma_list_error);

	for (int i = 0; i < DVI_N_TMDS_BUFFERS; ++i) {
		void *tmdsbuf;
#if DVI_MONOCHROME_TMDS
		tmdsbuf = malloc(inst->timing->h_active_pixels / DVI_SYMBOLS_PER_WORD * sizeof(uint32_t));
#else
		tmdsbuf = malloc(3 * inst->timing->h_active_pixels / DVI_SYMBOLS_PER_WORD * sizeof(uint32_t));
#endif
		if (!tmdsbuf)
			panic("TMDS buffer allocation failed");
		queue_add_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
	}
}

// The IRQs will run on whichever core calls this function (this is why it's
// called separately from dvi_init)
void dvi_register_irqs_this_core(struct dvi_inst *inst, uint irq_num) {
	uint32_t mask_sync_channel = 1u << inst->dma_cfg[TMDS_SYNC_LANE].chan_data;
	uint32_t mask_all_channels = 0;
	for (int i = 0; i < N_TMDS_LANES; ++i)
		mask_all_channels |= 1u << inst->dma_cfg[i].chan_ctrl | 1u << inst->dma_cfg[i].chan_data;

	dma_hw->ints0 = mask_sync_channel;
	if (irq_num == DMA_IRQ_0) {
		hw_write_masked(&dma_hw->inte0, mask_sync_channel, mask_all_channels);
		dma_irq_privdata[0] = inst;
		irq_set_exclusive_handler(DMA_IRQ_0, dvi_dma0_irq);
	}
	else {
		hw_write_masked(&dma_hw->inte1, mask_sync_channel, mask_all_channels);
		dma_irq_privdata[1] = inst;
		irq_set_exclusive_handler(DMA_IRQ_1, dvi_dma1_irq);
	}
	irq_set_enabled(irq_num, true);
}

// Set up control channels to make transfers to data channels' control
// registers (but don't trigger the control channels -- this is done either by
// data channel CHAIN_TO or an initial write to MULTI_CHAN_TRIGGER)
static inline void __attribute__((always_inline)) _dvi_load_dma_op(const struct dvi_lane_dma_cfg dma_cfg[], struct dvi_scanline_dma_list *l) {
	for (int i = 0; i < N_TMDS_LANES; ++i) {
		dma_channel_config cfg = dma_channel_get_default_config(dma_cfg[i].chan_ctrl);
		channel_config_set_ring(&cfg, true, 4); // 16-byte write wrap
		channel_config_set_read_increment(&cfg, true);
		channel_config_set_write_increment(&cfg, true);
		dma_channel_configure(
			dma_cfg[i].chan_ctrl,
			&cfg,
			&dma_hw->ch[dma_cfg[i].chan_data],
			dvi_lane_from_list(l, i),
			4, // Configure all 4 registers then halt until next CHAIN_TO
			false
		);
	}
}

// Setup first set of control block lists, configure the control channels, and
// trigger them. Control channels will subsequently be triggered only by DMA
// CHAIN_TO on data channel completion. IRQ handler *must* be prepared before
// calling this. (Hooked to DMA IRQ0)
void dvi_start(struct dvi_inst *inst) {
	_dvi_load_dma_op(inst->dma_cfg, &inst->dma_list_vblank_nosync);
	dma_start_channel_mask(
		(1u << inst->dma_cfg[0].chan_ctrl) |
		(1u << inst->dma_cfg[1].chan_ctrl) |
		(1u << inst->dma_cfg[2].chan_ctrl));

	// We really don't want the FIFOs to bottom out, so wait for full before
	// starting the shift-out.
	for (int i = 0; i < N_TMDS_LANES; ++i)
		while (!pio_sm_is_tx_fifo_full(inst->ser_cfg.pio, inst->ser_cfg.sm_tmds[i]))
			tight_loop_contents();
	dvi_serialiser_enable(&inst->ser_cfg, true);
}


static inline void __dvi_func_x(_dvi_prepare_scanline_8bpp)(struct dvi_inst *inst, uint32_t *scanbuf) {
	uint32_t *tmdsbuf;
	queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
	uint pixwidth = inst->timing->h_active_pixels;
	uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
	// Scanline buffers are half-resolution; the functions take the number of *input* pixels as parameter.
	tmds_encode_data_channel_8bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_8BPP_BLUE_MSB,  DVI_8BPP_BLUE_LSB );
	tmds_encode_data_channel_8bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_8BPP_GREEN_MSB, DVI_8BPP_GREEN_LSB);
	tmds_encode_data_channel_8bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_8BPP_RED_MSB,   DVI_8BPP_RED_LSB  );
	queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
}

static inline void __dvi_func_x(_dvi_prepare_scanline_16bpp)(struct dvi_inst *inst, uint32_t *scanbuf) {
	uint32_t *tmdsbuf;
	queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
	uint pixwidth = inst->timing->h_active_pixels;
	uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_16BPP_BLUE_MSB,  DVI_16BPP_BLUE_LSB );
	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_16BPP_GREEN_MSB, DVI_16BPP_GREEN_LSB);
	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_16BPP_RED_MSB,   DVI_16BPP_RED_LSB  );
	queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
}

static inline void __dvi_func_x(_dvi_prepare_scanline_12bpp)(struct dvi_inst *inst, uint32_t *scanbuf) {
	uint32_t *tmdsbuf;
	queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
	uint pixwidth = inst->timing->h_active_pixels;
	uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;

	
	//const uint pixwidth = 640;
	/*tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_12BPP_BLUE_MSB,  DVI_12BPP_BLUE_LSB );
	tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_12BPP_GREEN_MSB, DVI_12BPP_GREEN_LSB);
	tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_12BPP_RED_MSB,   DVI_12BPP_RED_LSB);*/

	/*tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth, DVI_12BPP_BLUE_MSB,  DVI_12BPP_BLUE_LSB );
	tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth, DVI_12BPP_GREEN_MSB, DVI_12BPP_GREEN_LSB);
	tmds_encode_data_channel_fullres_16bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth, DVI_12BPP_RED_MSB,   DVI_12BPP_RED_LSB);*/

	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_12BPP_BLUE_MSB,  DVI_12BPP_BLUE_LSB );
	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_12BPP_GREEN_MSB, DVI_12BPP_GREEN_LSB);
	tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_12BPP_RED_MSB,   DVI_12BPP_RED_LSB  );

	/*tmds_encode_data_channel_12bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_12BPP_BLUE_MSB,  DVI_12BPP_BLUE_LSB );
	tmds_encode_data_channel_12bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_12BPP_GREEN_MSB, DVI_12BPP_GREEN_LSB);
	tmds_encode_data_channel_12bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_12BPP_RED_MSB,   DVI_12BPP_RED_LSB  );*/

	/*tmds_encode_data_channel_12bpp_2(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_12BPP_BLUE_MSB,  DVI_12BPP_BLUE_LSB );
	tmds_encode_data_channel_12bpp_2(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_12BPP_GREEN_MSB, DVI_12BPP_GREEN_LSB);
	tmds_encode_data_channel_12bpp_2(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_12BPP_RED_MSB,   DVI_12BPP_RED_LSB  );*/


	queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
}


// "Worker threads" for TMDS encoding (core enters and never returns, but still handles IRQs)

// Version where each record in q_colour_valid is one scanline:
void __dvi_func(dvi_scanbuf_main_8bpp)(struct dvi_inst *inst) {
	uint y = 0;
	while (1) {
		uint32_t *scanbuf;
		queue_remove_blocking_u32(&inst->q_colour_valid, &scanbuf);
		_dvi_prepare_scanline_8bpp(inst, scanbuf);
		queue_add_blocking_u32(&inst->q_colour_free, &scanbuf);
		++y;
		if (y == inst->timing->v_active_lines) {
			y = 0;
		}
	}
	__builtin_unreachable();
}

// Ugh copy/paste but it lets us garbage collect the TMDS stuff that is not being used from .scratch_x
void __dvi_func(dvi_scanbuf_main_16bpp)(struct dvi_inst *inst) {
	uint y = 0;
	while (1) {
		uint32_t *scanbuf;
		queue_remove_blocking_u32(&inst->q_colour_valid, &scanbuf);
		_dvi_prepare_scanline_16bpp(inst, scanbuf);
		queue_add_blocking_u32(&inst->q_colour_free, &scanbuf);
		++y;
		if (y == inst->timing->v_active_lines) {
			y = 0;
		}
	}
	__builtin_unreachable();
}



// Ugh copy/paste but it lets us garbage collect the TMDS stuff that is not being used from .scratch_x
void __dvi_func(dvi_scanbuf_main_12bpp)(struct dvi_inst *inst) {
	uint y = 0;
	while (1) {
		uint32_t *scanbuf;
		queue_remove_blocking_u32(&inst->q_colour_valid, &scanbuf);
		_dvi_prepare_scanline_12bpp(inst, scanbuf);
		queue_add_blocking_u32(&inst->q_colour_free, &scanbuf);
		++y;
		if (y == inst->timing->v_active_lines) {
			y = 0;
		}
	}
	__builtin_unreachable();
}



uint32_t scanbuf_pointer = 0;
uint16_t empty_scanline[640];

//uint16_t * scanbuf = (uint16_t *)(0x20000000 + (1024 * 30));
//uint16_t * scanbuf2 = (uint16_t *)(0x20000000 + (pixels_in_scanline * scanlines_in_active_area * 2) + (1024 * 30));

//uint16_t scanbuf[pixels_in_scanline * scanlines_in_active_area];
//uint16_t scanbuf2[pixels_in_scanline * scanlines_in_active_area];

// Ugh copy/paste but it lets us garbage collect the TMDS stuff that is not being used from .scratch_x
void __dvi_func(dvi_scanbuf_main_12bpp_noqueue)(struct dvi_inst *inst, uint16_t *scanbuf) {
	uint16_t y = 0;
	uint16_t * curr_framebuffer;

	//if(dma_channel_is_busy(dma_chan0)) curr_framebuffer = framebuffer2;
	//else curr_framebuffer = framebuffer;
	curr_framebuffer = scanbuf;

	bool frame_tail = false;
	while (1) {
		//if(y > header_scanlines && y < (scanlines_in_active_area + header_scanlines) - 1) {
		if(y < scanlines_in_active_area) {

			//const uint16_t *scanline = &curr_framebuffer[y * pixels_in_scanline];
			//_dvi_prepare_scanline_12bpp(inst, (uint32_t *) &scanline);
			uint16_t buf;
			uint16_t pix;
			//uint8_t r,g,b;
			uint16_t t1,t2,t3,t4;
			for(uint32_t c = 0; c < 640; c++) 
			{
				if(c < pixels_in_scanline)
				{
					pix = 0;
					buf = curr_framebuffer[scanbuf_pointer];

					/*pix |=  buf & 0x1 << 3;
					pix |=  buf & 0x2 << 1;
					pix |=  buf & 0x4 >> 1;
					pix |=  buf & 0x8 >> 3;*/

					//b = buf & 0xF; //4 bit blue
					//g = (buf >> 4) & 0xF; //4 bit green
					//r = (buf >> 8) & 0xF; //4 bit red
/*
					t1 = (r & 0x8) >> 3;
					t2 = (r & 0x4) >> 1;
					t3 = (r & 0x2) << 1;
					t4 = (r & 0x1) << 3;
					
					r = t1 | t2 | t3 | t4;

					t1 = (g & 0x8) >> 3;
					t2 = (g & 0x4) >> 1;
					t3 = (g & 0x2) << 1;
					t4 = (g & 0x1) << 3;
					
					g = t1 | t2 | t3 | t4;
*/
					/*t1 = (buf >> 3 & 0x1);
					t2 = (buf >> 1 & 0x2);
					t3 = (buf << 1 & 0x4);
					t4 = (buf << 3 & 0x8);*/
					
					//b = (b >> 3 & 0x1) | (b >> 1 & 0x2) | (b << 1 & 0x4) | (b << 3 & 0x8);
					//b = t1 | t2 | t3 | t4;

					
					buf = buf << 3;
					t1 = buf  & 0x888;
					t2 = (buf >> 2) & 0x444;
					t3 = (buf >> 4) & 0x222;
					t3 = (buf >> 6) & 0x111;

/*
					t1 = (buf << 3) & 0x111;
					t2 = (buf << 1) & 0x222;
					t3 = (buf >> 1) & 0x444;
					t4 = (buf >> 3) & 0x888;*/

					empty_scanline[c] = t1 | t2 | t3 | t4;

					//buf =  r << 8 | g << 4 | b;

					//empty_scanline[c] = r << 8 | g << 4 | b;//  << 5;


					//empty_scanline[c] = buf;
					
					//empty_scanline[c] = (gamma4[(buf >> 8) & 0xF] << 12) | (gamma4[(buf >> 4) & 0xF] << 6) | (gamma4[(buf) & 0xF] << 0);
/*
					empty_scanline[c] = 
						((buf & 0xF00)  << 4 ) | //r
						((buf & 0xF0)  << 3 ) | //g
						((buf & 0xF) << 1 )  //b
						;
*/
					/*
						((curr_framebuffer[scanbuf_pointer + 1] & 0xF00)  << 21 ) | //r
						((curr_framebuffer[scanbuf_pointer + 1] & 0xF0)  << 18 ) | //g
						((curr_framebuffer[scanbuf_pointer + 1] & 0xF) << 17 );  //b
						*/
				
					//scanbuf_pointer+=2;
					scanbuf_pointer++;
				}
				/*if(c > pixels_border && c < pixels_in_scanline + pixels_border +1) {
					empty_scanline[c] = curr_framebuffer[scanbuf_pointer];
					scanbuf_pointer++;
				}*/
				else empty_scanline[c] = 0;
			}
			
			//_dvi_prepare_scanline_12bpp(inst, (uint32_t *) &empty_scanline);
		}
		else {
			if(!frame_tail)
			{
				for(uint32_t c = 0; c < 640; c++) {
					empty_scanline[c] = 0;
				}
				/*for(uint32_t c = 0; c < 320; c++) {
					empty_scanline[c] = 0;
				}*/
				frame_tail = true;
			}
			//_dvi_prepare_scanline_12bpp(inst, (uint32_t *) &empty_scanline);
		}

		_dvi_prepare_scanline_12bpp(inst, (uint32_t *) &empty_scanline[0]);
		//_dvi_prepare_scanline_16bpp(inst, (uint32_t *) &empty_scanline[0]);
		//_dvi_prepare_scanline_16bpp(inst, (uint32_t *) &empty_scanline[0]);

		++y;
		if (y == (inst->timing->v_active_lines >> 1)) {
			//gpio_put(25, !gpio_get(25));
			y = 0;
			scanbuf_pointer = 0;
			frame_tail = false;
		}
	}
	__builtin_unreachable();
}

static void __dvi_func(dvi_dma_irq_handler)(struct dvi_inst *inst) {
	// Every fourth interrupt marks the start of the horizontal active region. We
	// now have until the end of this region to generate DMA blocklist for next
	// scanline.
	dvi_timing_state_advance(inst->timing, &inst->timing_state);
	if (inst->tmds_buf_release && !queue_try_add_u32(&inst->q_tmds_free, &inst->tmds_buf_release))
		panic("TMDS free queue full in IRQ!");
	inst->tmds_buf_release = inst->tmds_buf_release_next;
	inst->tmds_buf_release_next = NULL;

	// Make sure all three channels have definitely loaded their last block
	// (should be within a few cycles of one another)
	for (int i = 0; i < N_TMDS_LANES; ++i) {
		while (dma_debug_hw->ch[inst->dma_cfg[i].chan_data].dbg_tcr != inst->timing->h_active_pixels / DVI_SYMBOLS_PER_WORD)
			tight_loop_contents();
	}

	uint32_t *tmdsbuf;
	while (inst->late_scanline_ctr > 0 && queue_try_remove_u32(&inst->q_tmds_valid, &tmdsbuf)) {
		// If we displayed this buffer then it would be in the wrong vertical
		// position on-screen. Just pass it back.
		queue_add_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
		--inst->late_scanline_ctr;
	}

	if (inst->timing_state.v_state != DVI_STATE_ACTIVE) {
		// Don't care
		tmdsbuf = NULL;
	}
	else if (queue_try_peek_u32(&inst->q_tmds_valid, &tmdsbuf)) {
		if (inst->timing_state.v_ctr % DVI_VERTICAL_REPEAT == DVI_VERTICAL_REPEAT - 1) {
			queue_remove_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
			inst->tmds_buf_release_next = tmdsbuf;
		}
	}
	else {
		// No valid scanline was ready (generates solid red scanline)
		tmdsbuf = NULL;
		if (inst->timing_state.v_ctr % DVI_VERTICAL_REPEAT == DVI_VERTICAL_REPEAT - 1)
			++inst->late_scanline_ctr;
	}

	switch (inst->timing_state.v_state) {
		case DVI_STATE_ACTIVE:
			if (tmdsbuf) {
				dvi_update_scanline_data_dma(inst->timing, tmdsbuf, &inst->dma_list_active);
				_dvi_load_dma_op(inst->dma_cfg, &inst->dma_list_active);
			}
			else {
				_dvi_load_dma_op(inst->dma_cfg, &inst->dma_list_error);
			}
			if (inst->scanline_callback && inst->timing_state.v_ctr % DVI_VERTICAL_REPEAT == DVI_VERTICAL_REPEAT - 1) {
				inst->scanline_callback();
			}
			break;
		case DVI_STATE_SYNC:
			_dvi_load_dma_op(inst->dma_cfg, &inst->dma_list_vblank_sync);
			break;
		default:
			_dvi_load_dma_op(inst->dma_cfg, &inst->dma_list_vblank_nosync);
			break;
	}
}

static void __dvi_func(dvi_dma0_irq)() {
	struct dvi_inst *inst = dma_irq_privdata[0];
	dma_hw->ints0 = 1u << inst->dma_cfg[TMDS_SYNC_LANE].chan_data;
	dvi_dma_irq_handler(inst);
}

static void __dvi_func(dvi_dma1_irq)() {
	struct dvi_inst *inst = dma_irq_privdata[1];
	dma_hw->ints1 = 1u << inst->dma_cfg[TMDS_SYNC_LANE].chan_data;
	dvi_dma_irq_handler(inst);
}
