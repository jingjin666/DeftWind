/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_mixer.h
 *
 * Mixer ioctl interfaces.
 *
 * Normal workflow is:
 *
 * - open mixer device
 * - add mixer(s)
 * loop:
 *  - mix actuators to array
 *
 * Each client has its own configuration.
 *
 * When mixing, outputs are produced by mixers in the order they are
 * added.  A simple mixer produces one output; a multirotor mixer will
 * produce several outputs, etc.
 */

#ifndef _DRV_MIXER_H
#define _DRV_MIXER_H

#include <dp_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define MIXER0_DEVICE_PATH		"/dev/mixer0"

/*
 * ioctl() definitions
 */
#define _MIXERIOCBASE		(0x2500)
#define _MIXERIOC(_n)		(_DP_IOC(_MIXERIOCBASE, _n))

/** get the number of mixable outputs */
#define MIXERIOCGETOUTPUTCOUNT	_MIXERIOC(0)

/** reset (clear) the mixer configuration */
#define MIXERIOCRESET		_MIXERIOC(1)

/** simple channel scaler */
struct mixer_scaler_s {
	float			negative_scale;
	float			positive_scale;
	float			offset;
	float			min_output;
	float			max_output;
};

/** mixer input */
struct mixer_control_s {
	uint8_t			control_group;	/**< group from which the input reads */
	uint8_t			control_index;	/**< index within the control group */
	struct mixer_scaler_s 	scaler;		/**< scaling applied to the input before use */
};

/** simple mixer */
struct mixer_simple_s {
	uint8_t			control_count;	/**< number of inputs */
	struct mixer_scaler_s	output_scaler;	/**< scaling for the output */
	struct mixer_control_s	controls[0];	/**< actual size of the array is set by control_count */
};

#define MIXER_SIMPLE_SIZE(_icount)	(sizeof(struct mixer_simple_s) + (_icount) * sizeof(struct mixer_control_s))

/**
 * add a simple mixer in (struct mixer_simple_s *)arg
 */
#define MIXERIOCADDSIMPLE	_MIXERIOC(2)

/* _MIXERIOC(3) was deprecated */
/* _MIXERIOC(4) was deprecated */

/**
 * Add mixer(s) from the buffer in (const char *)arg
 */
#define MIXERIOCLOADBUF		_MIXERIOC(5)

/*
 * XXX Thoughts for additional operations:
 *
 * - get/set output scale, for tuning center/limit values.
 * - save/serialise for saving tuned mixers.
 */

#endif /* _DRV_ACCEL_H */
