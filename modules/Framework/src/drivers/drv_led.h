/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_led.h
 *
 * LED driver API
 */

#pragma once

#include <dp_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define LED_BASE_DEVICE_PATH		"/dev/led"
#define LED0_DEVICE_PATH		"/dev/led0"

#define _LED_BASE		0x2800

/* DP LED colour codes */
#define LED_WORKSTATUS		1


#define LED_ON			_DP_IOC(_LED_BASE, 0)
#define LED_OFF			_DP_IOC(_LED_BASE, 1)
#define LED_TOGGLE		_DP_IOC(_LED_BASE, 2)

__BEGIN_DECLS

/*
 * Initialise the LED driver.
 */
__EXPORT void drv_led_start(void);

__END_DECLS
