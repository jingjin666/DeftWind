/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_micro_hal.h
   Configuration flags used in code.
 */

#pragma once

__BEGIN_DECLS

#ifdef __DP_NUTTX
#define dp_enter_critical_section()       		enter_critical_section()
#define dp_leave_critical_section(flags)  	leave_critical_section(flags)

#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)
include <stm32.h>
#define dp_arch_configgpio(pinset)             		stm32_configgpio(pinset)
#define dp_arch_unconfiggpio(pinset)           	stm32_unconfiggpio(pinset)
#define dp_arch_gpioread(pinset)               		stm32_gpioread(pinset)
#define dp_arch_gpiowrite(pinset, value)       	stm32_gpiowrite(pinset, value)
#define dp_arch_gpiosetevent(pinset,r,f,e,fp,a)   stm32_gpiosetevent(pinset,r,f,e,fp,a)

#include <stm32_i2c.h>
#define dp_i2cbus_initialize(bus_num_1based)		stm32_i2cbus_initialize(bus_num_1based)
#define dp_i2cbus_uninitialize(pdev)				stm32_i2cbus_uninitialize(pdev)

#include <stm32_spi.h>
#define dp_spibus_initialize(bus_num_1based)		stm32_spibus_initialize(bus_num_1based)
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
#include <chip.h>
#define dp_arch_configgpio(pinset)             		imxrt_config_gpio(pinset)
#define dp_arch_unconfiggpio(pinset)
#define dp_arch_gpioread(pinset)              	 	imxrt_gpio_read(pinset)
#define dp_arch_gpiowrite(pinset, value)       	imxrt_gpio_write(pinset, value)
#define dp_arch_gpiosetevent(pinset,r,f,e,fp,a)

#include <imxrt_lpi2c.h>
#define dp_i2cbus_initialize(bus_num_1based)   	imxrt_i2cbus_initialize(bus_num_1based)
#define dp_i2cbus_uninitialize(pdev)           			imxrt_i2cbus_uninitialize(pdev)

#include <imxrt_lpspi.h>
#define dp_spibus_initialize(bus_num_1based)		imxrt_lpspibus_initialize(bus_num_1based)
#endif

#endif
__END_DECLS