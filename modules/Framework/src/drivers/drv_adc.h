/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_adc.h
 *
 * ADC driver interface.
 *
 * This defines additional operations over and above the standard NuttX
 * ADC API.
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* Define the DP low level format ADC and the maximum
 * number of channels that can be returned by a lowlevel
 * ADC driver. Drivers may return less than DP_MAX_ADC_CHANNELS
 * but no more than DP_MAX_ADC_CHANNELS.
 *
 */
#define DP_MAX_ADC_CHANNELS 12
typedef struct __attribute__((packed)) dp_adc_msg_t {
	uint8_t      am_channel;               /* The 8-bit ADC Channel */
	int32_t      am_data;                  /* ADC convert result (4 bytes) */
} dp_adc_msg_t;


#define ADC0_DEVICE_PATH	"/dev/adc0"

/*
 * ioctl definitions
 */
