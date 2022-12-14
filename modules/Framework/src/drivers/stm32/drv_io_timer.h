/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_io_timer.h
 *
 * stm32-specific PWM output data.
 */
#include <dp_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <drivers/drv_hrt.h>

#pragma once
__BEGIN_DECLS
/* configuration limits */
#define MAX_IO_TIMERS			4
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54) || defined(CONFIG_ARCH_BOARD_VRCORE_V10) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52) || defined(CONFIG_ARCH_BOARD_UAVRS_V1)
#define MAX_TIMER_IO_CHANNELS	12
#else
#define MAX_TIMER_IO_CHANNELS	8
#endif
#define IO_TIMER_ALL_MODES_CHANNELS 0

typedef enum io_timer_channel_mode_t {
	IOTimerChanMode_NotUsed = 0,
	IOTimerChanMode_PWMOut  = 1,
	IOTimerChanMode_PWMIn   = 2,
	IOTimerChanMode_Capture = 3,
	IOTimerChanModeSize
} io_timer_channel_mode_t;

typedef uint16_t io_timer_channel_allocation_t; /* big enough to hold MAX_TIMER_IO_CHANNELS */

/* array of timers dedicated to PWM in and out and capture use */
typedef struct io_timers_t {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	clock_freq;
	uint32_t	vectorno;
	uint32_t    first_channel_index;
	uint32_t    last_channel_index;
	xcpt_t      handler;
} io_timers_t;

/* array of channels in logical order */
typedef struct timer_io_channels_t {
	uint32_t	gpio_out;
	uint32_t	gpio_in;
	uint8_t		timer_index;
	uint8_t		timer_channel;
	uint16_t	masks;
	uint8_t		ccr_offset;
} timer_io_channels_t;


typedef void (*channel_handler_t)(void *context, const io_timers_t *timer, uint32_t chan_index,
				  const timer_io_channels_t *chan,
				  hrt_abstime isrs_time , uint16_t isrs_rcnt);


/* supplied by board-specific code */
__EXPORT extern const io_timers_t io_timers[MAX_IO_TIMERS];
__EXPORT extern const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS];
__EXPORT extern io_timer_channel_allocation_t allocations[IOTimerChanModeSize];
__EXPORT int io_timer_handler0(int irq, void *context, void *arg);
__EXPORT int io_timer_handler1(int irq, void *context, void *arg);
__EXPORT int io_timer_handler2(int irq, void *context, void *arg);
__EXPORT int io_timer_handler3(int irq, void *context, void *arg);

__EXPORT int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
				   channel_handler_t channel_handler, void *context);
__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT int io_timer_set_enable(bool state, io_timer_channel_mode_t mode,
				 io_timer_channel_allocation_t masks);
__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT int io_timer_set_clock(unsigned timer, unsigned clock_MHz);
__EXPORT uint16_t io_channel_get_ccr(unsigned channel);
__EXPORT int io_timer_set_ccr(unsigned channel, uint16_t value);
__EXPORT uint32_t io_timer_get_group(unsigned timer);
__EXPORT int io_timer_pwm_trigger(uint32_t channel_mask);
__EXPORT int io_timer_validate_channel_index(unsigned channel);
__EXPORT int io_timer_is_channel_free(unsigned channel);
__EXPORT int io_timer_free_channel(unsigned channel);
__EXPORT int io_timer_get_channel_mode(unsigned channel);
__EXPORT int io_timer_get_mode_channels(io_timer_channel_mode_t mode);
__END_DECLS
