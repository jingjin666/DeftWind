/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_rc_input.h
 *
 * R/C input interface.
 */

#ifndef _DRV_RC_INPUT_H
#define _DRV_RC_INPUT_H

#include <stdint.h>
#include <sys/ioctl.h>
#include <uORB/topics/input_rc.h>

#include "drv_orb_dev.h"

/**
 * Path for the default R/C input device.
 *
 * Note that on systems with more than one R/C input path (e.g.
 * PX4FMU with PX4IO connected) there may be other devices that
 * respond to this protocol.
 *
 * Input data may be obtained by subscribing to the input_rc
 * object, or by poll/reading from the device.
 */
#define RC_INPUT0_DEVICE_PATH	"/dev/input_rc0"

/**
 * Maximum RSSI value
 */
#define RC_INPUT_RSSI_MAX	100

/**
 * Minimum value
 */
#define RC_INPUT_LOWEST_MIN_US	500

/**
 * Maximum value
 */
#define RC_INPUT_HIGHEST_MAX_US	2500

/**
 * Maximum deadzone value
 */
#define RC_INPUT_MAX_DEADZONE_US	500

#include <uORB/topics/input_rc.h>
#define rc_input_values input_rc_s

/**
 * Input signal type, value is a control position from zero to 100
 * percent.
 */
typedef uint16_t		rc_input_t;

#define _RC_INPUT_BASE		0x2b00

/** Fetch R/C input values into (rc_input_values *)arg */
#define RC_INPUT_GET			_IOC(_RC_INPUT_BASE, 0)

/** Enable RSSI input via ADC */
#define RC_INPUT_ENABLE_RSSI_ANALOG	_IOC(_RC_INPUT_BASE, 1)

/** Enable RSSI input via PWM signal */
#define RC_INPUT_ENABLE_RSSI_PWM	_IOC(_RC_INPUT_BASE, 2)

#endif /* _DRV_RC_INPUT_H */
