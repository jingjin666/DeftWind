/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file Input capture interface.
 *
 * Input capture interface utilizes the FMU_AUX_PINS to time stamp
 * an edge.
 */

#pragma once

#include <dp_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_hrt.h"

__BEGIN_DECLS

/**
 * Path for the default capture input device.
 *
 *
 */
#define INPUT_CAPTURE_BASE_DEVICE_PATH "/dev/capture"
#define INPUT_CAPTURE0_DEVICE_PATH	"/dev/capture0"

typedef void (*capture_callback_t)(void *context, uint32_t chan_index,
				   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

/**
 * Maximum number of PWM input channels supported by the device.
 */
#ifndef INPUT_CAPTURE_MAX_CHANNELS
#define INPUT_CAPTURE_MAX_CHANNELS 6
#endif

typedef uint16_t capture_filter_t;
typedef uint16_t capture_t;

typedef enum input_capture_edge {
	Disabled 	= 	0,
	Rising 	 	=   1,
	Falling		=   2,
	Both		=   3
} input_capture_edge;

typedef struct input_capture_element_t {
	hrt_abstime			time_stamp;
	input_capture_edge	edge;
	bool				overrun;
} input_capture_element_t;

typedef struct input_capture_stats_t {
	uint32_t 		   chan_in_edges_out;
	uint32_t 		   overflows;
	uint32_t		   last_edge;
	hrt_abstime		   last_time;
	uint16_t		   latnecy;
} input_capture_stats_t;

/**
 * input capture values for a channel
 *
 * This allows for Capture input driver values to be set without a
 * param_get() dependency
 */
typedef struct input_capture_config_t {
	uint8_t 		channel;
	capture_filter_t filter;
	input_capture_edge edge;
	capture_callback_t callback;
	void			   *context;

} input_capture_config_t;

/*
 * ioctl() definitions
 *
 * Note that ioctls and ORB updates should not be mixed, as the
 * behaviour of the system in this case is not defined.
 */
#define _INPUT_CAP_BASE		0x2d00

/** Set Enable a channel arg is pointer to input_capture_config
 * with all parameters set.
 *  edge controls the mode: Disable will free the capture channel.
 *  (When edge is Disabled call back and context are ignored)
 *  context may be null. If callback and context are null the
 *  callback will be disabled.
 *  */

#define INPUT_CAP_SET			_DP_IOC(_INPUT_CAP_BASE, 0)

/** Set the call back on a capture channel - arg is pointer to
 * input_capture_config with channel call back and context set
 * context may be null. If both ate null the call back will be
 * disabled  */
#define INPUT_CAP_SET_CALLBACK	_DP_IOC(_INPUT_CAP_BASE, 1)

/** Get the call back on a capture channel - arg is pointer to
 * input_capture_config with channel set.
 */
#define INPUT_CAP_GET_CALLBACK	_DP_IOC(_INPUT_CAP_BASE, 2)

/** Set Edge a channel  arg is pointer to input_capture_config
 * with channel and edge set */
#define INPUT_CAP_SET_EDGE		_DP_IOC(_INPUT_CAP_BASE, 3)

/** Get Edge for a channel   arg is pointer to input_capture_config
 * with channel set */
#define INPUT_CAP_GET_EDGE		_DP_IOC(_INPUT_CAP_BASE, 4)

/** Set Filter input filter channel    arg is pointer to input_capture_config
 * with channel and filter set */
#define INPUT_CAP_SET_FILTER	_DP_IOC(_INPUT_CAP_BASE, 5)

/** Set Filter input filter channel   arg is pointer to input_capture_config
 * with channel set */
#define INPUT_CAP_GET_FILTER	_DP_IOC(_INPUT_CAP_BASE, 6)

/** Get the number of capture in *(unsigned *)arg */
#define INPUT_CAP_GET_COUNT		_DP_IOC(_INPUT_CAP_BASE, 7)

/** Set the number of capture in (unsigned)arg - allows change of
 * split between servos and capture  */
#define INPUT_CAP_SET_COUNT		_DP_IOC(_INPUT_CAP_BASE, 8)

/** Get channel stats - arg is pointer to input_capture_config
 * with channel set.
 */
#define INPUT_CAP_GET_STATS		_DP_IOC(_INPUT_CAP_BASE, 9)

/** Get channel stats - arg is pointer to input_capture_config
 * with channel set.
 */
#define INPUT_CAP_GET_CLR_STATS	_DP_IOC(_INPUT_CAP_BASE, 10)

/*
 *
 *
 * WARNING WARNING WARNING! DO NOT EXCEED 31 IN IOC INDICES HERE!
 *
 *
 */

__EXPORT int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
				  capture_callback_t callback, void *context);

__EXPORT int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter);
__EXPORT int up_input_capture_set_filter(unsigned channel,  capture_filter_t filter);

__EXPORT int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge);
__EXPORT int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge);
__EXPORT int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context);
__EXPORT int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context);
__EXPORT int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear);
__END_DECLS
