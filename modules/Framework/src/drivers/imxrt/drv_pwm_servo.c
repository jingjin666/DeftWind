/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to FLexPWM timer blocks.
 * N.B. Groups:channels have a 1:1 correspondence on FlexPWM
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
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"
#include "drv_pwm_servo.h"

//#include <chip.h>

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	return io_timer_set_ccr(channel, value);
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	return io_channel_get_ccr(channel);
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	/* Init channels */
	uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_PWMOut);

	// First free the current set of PWMs

	for (unsigned channel = 0; current != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (current & (1 << channel)) {
			io_timer_free_channel(channel);
			current &= ~(1 << channel);
		}
	}

	// Now allocate the new set

	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {

			// First free any that were not PWM mode before

			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);
			channel_mask &= ~(1 << channel);
		}
	}

	return OK;
}

void up_pwm_servo_deinit(void)
{
	/* disable the timers */
	up_pwm_servo_arm(false);
}

int up_pwm_servo_set_rate_group_update(unsigned channel, unsigned rate)
{
	if (io_timer_validate_channel_index(channel) < 0) {
		return ERROR;
	}

	/* Allow a rate of 0 to enter oneshot mode */

	if (rate != 0) {

		/* limit update rate to 1..10000Hz; somewhat arbitrary but safe */

		if (rate < 1) {
			return -ERANGE;
		}

		if (rate > 10000) {
			return -ERANGE;
		}
	}

	return io_timer_set_rate(channel, rate);
}

void up_pwm_update(void)
{
	io_timer_trigger();
}

int up_pwm_servo_set_rate(unsigned rate)
{
	for (unsigned i = 0; i < MAX_TIMER_IO_CHANNELS; i++) {
		up_pwm_servo_set_rate_group_update(i, rate);
	}

	return 0;
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return io_timer_get_group(group);
}

void
up_pwm_servo_arm(bool armed)
{
	io_timer_set_enable(armed, IOTimerChanMode_OneShot, IO_TIMER_ALL_MODES_CHANNELS);
	io_timer_set_enable(armed, IOTimerChanMode_PWMOut, IO_TIMER_ALL_MODES_CHANNELS);
}
