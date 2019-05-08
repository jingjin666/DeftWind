/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/*
 * @file drv_input_capture.c
 *
 * Servo driver supporting input capture connected to imxrt  timer blocks.
 *
 * Works with any FLEXPWN that have input pins.
 *
 * Require an interrupt.
 *
 *
 */

#include <dp_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_input_capture.h>
#include "drv_io_timer.h"

#include "drv_input_capture.h"

#include <chip.h>
#include "chip/imxrt_flexpwm.h"

#define MAX_CHANNELS_PER_TIMER 2

#define SM_SPACING (IMXRT_FLEXPWM_SM1CNT_OFFSET-IMXRT_FLEXPWM_SM0CNT_OFFSET)

#define _REG(_addr) (*(volatile uint16_t *)(_addr))
#define _REG16(_base, _reg) (*(volatile uint16_t *)(_base + _reg))
#define REG(_tmr, _sm, _reg)    _REG16(io_timers[(_tmr)].base + ((_sm) * SM_SPACING), (_reg))

static input_capture_stats_t channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	capture_callback_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];

static void input_capture_chan_handler(void *context, const io_timers_t *timer, uint32_t chan_index,
				       const timer_io_channels_t *chan,
				       hrt_abstime isrs_time, uint16_t isrs_rcnt,
				       uint16_t capture)
{
	channel_stats[chan_index].last_edge = dp_arch_gpioread(chan->gpio_in);

	if ((isrs_rcnt - capture) > channel_stats[chan_index].latnecy) {
		channel_stats[chan_index].latnecy = (isrs_rcnt - capture);
	}

	channel_stats[chan_index].chan_in_edges_out++;
	channel_stats[chan_index].last_time = isrs_time - (isrs_rcnt - capture);
	uint32_t overflow = 0;//_REG32(timer, KINETIS_FTM_CSC_OFFSET(chan->timer_channel - 1)) & FTM_CSC_CHF;

	if (overflow) {

		/* Error we has a second edge before we cleared CCxR */

		channel_stats[chan_index].overflows++;
	}

	if (channel_handlers[chan_index].callback)  {
		channel_handlers[chan_index].callback(channel_handlers[chan_index].context, chan_index,
						      channel_stats[chan_index].last_time,
						      channel_stats[chan_index].last_edge, overflow);
	}
}

static void input_capture_bind(unsigned channel, capture_callback_t callback, void *context)
{
	irqstate_t flags = dp_enter_critical_section();
	channel_handlers[channel].callback = callback;
	channel_handlers[channel].context = context;
	dp_leave_critical_section(flags);
}

static void input_capture_unbind(unsigned channel)
{
	input_capture_bind(channel, NULL, NULL);
}

int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
			 capture_callback_t callback, void *context)
{
	if (edge > Both) {
		return -EINVAL;
	}

	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		if (edge == Disabled) {

			io_timer_set_enable(false, IOTimerChanMode_Capture, 1 << channel);
			input_capture_unbind(channel);

		} else {

			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			input_capture_bind(channel, callback, context);

			rv = io_timer_channel_init(channel, IOTimerChanMode_Capture, input_capture_chan_handler, context);

			if (rv != 0) {
				return rv;
			}

			rv = up_input_capture_set_filter(channel, filter);

			if (rv == 0) {
				rv = up_input_capture_set_trigger(channel, edge);

				if (rv == 0) {
					rv = io_timer_set_enable(true, IOTimerChanMode_Capture, 1 << channel);
				}
			}
		}
	}

	return rv;
}

int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter)
{
	return 0;
}
int up_input_capture_set_filter(unsigned channel,  capture_filter_t filter)
{
	return 0;
}

int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			rv = OK;

			uint32_t timer = timer_io_channels[channel].timer_index;
			uint32_t offset =  timer_io_channels[channel].val_offset == PWMA_VAL ? IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET :
					   IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET;
			uint32_t rvalue = REG(timer, timer_io_channels[channel].sub_module, offset);
			rvalue &= SMC_EDGA0_BOTH;

			switch (rvalue) {

			case (SMC_EDGA0_RISING):
				*edge = Rising;
				break;

			case (SMC_EDGA0_FALLING):
				*edge = Falling;
				break;

			case (SMC_EDGA0_BOTH):
				*edge = Both;
				break;

			default:
				rv = -EIO;
			}
		}
	}

	return rv;
}
int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			uint16_t edge_bits = 0;

			switch (edge) {
			case Disabled:
				break;

			case Rising:
				edge_bits = SMC_EDGA0_RISING;
				break;

			case Falling:
				edge_bits = SMC_EDGA0_FALLING;
				break;

			case Both:
				edge_bits = SMC_EDGA0_BOTH;
				break;

			default:
				return -EINVAL;;
			}

			uint32_t timer = timer_io_channels[channel].timer_index;
			uint32_t offset =  timer_io_channels[channel].val_offset == PWMA_VAL ? IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET :
					   IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET;
			irqstate_t flags = dp_enter_critical_section();
			uint32_t rvalue = REG(timer, timer_io_channels[channel].sub_module, offset);
			rvalue &= ~SMC_EDGA0_BOTH;
			rvalue |=  edge_bits;
			REG(timer, timer_io_channels[channel].sub_module, offset) = rvalue;
			dp_leave_critical_section(flags);
			rv = OK;
		}
	}

	return rv;
}

int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			irqstate_t flags = dp_enter_critical_section();
			*callback = channel_handlers[channel].callback;
			*context = channel_handlers[channel].context;
			dp_leave_critical_section(flags);
			rv = OK;
		}
	}

	return rv;

}

int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {
			input_capture_bind(channel, callback, context);
			rv = 0;
		}
	}

	return rv;
}

int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		irqstate_t flags = dp_enter_critical_section();
		*stats =  channel_stats[channel];

		if (clear) {
			memset(&channel_stats[channel], 0, sizeof(*stats));
		}

		dp_leave_critical_section(flags);
	}

	return rv;
}
