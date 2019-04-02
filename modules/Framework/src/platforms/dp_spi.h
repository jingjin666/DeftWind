/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_spi.h
 *
 * Includes device headers depending on the build target
 */

#pragma once

#ifdef __DP_NUTTX
#include <nuttx/spi/spi.h>
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)
	#include <stm32_spi.h>
	#define dp_spibus_initialize(bus_num_1based)		stm32_spibus_initialize(bus_num_1based)
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
	#include <imxrt_lpspi.h>
	#define dp_spibus_initialize(bus_num_1based)		imxrt_lpspibus_initialize(bus_num_1based)
#endif
#elif defined(__DP_POSIX)
enum spi_dev_e {
	SPIDEV_NONE = 0,    /* Not a valid value */
	SPIDEV_MMCSD,       /* Select SPI MMC/SD device */
	SPIDEV_FLASH,       /* Select SPI FLASH device */
	SPIDEV_ETHERNET,    /* Select SPI ethernet device */
	SPIDEV_DISPLAY,     /* Select SPI LCD/OLED display device */
	SPIDEV_WIRELESS,    /* Select SPI Wireless device */
	SPIDEV_TOUCHSCREEN, /* Select SPI touchscreen device */
	SPIDEV_EXPANDER,    /* Select SPI I/O expander device */
	SPIDEV_MUX,         /* Select SPI multiplexer device */
	SPIDEV_AUDIO_DATA,  /* Select SPI audio codec device data port */
	SPIDEV_AUDIO_CTRL,  /* Select SPI audio codec device control port */
	SPIDEV_MISC_1,
	SPIDEV_MISC_2,
	SPIDEV_MISC_3,
	SPIDEV_MISC_4
};

/* Certain SPI devices may required different clocking modes */

enum spi_mode_e {
	SPIDEV_MODE0 = 0,   /* CPOL=0 CHPHA=0 */
	SPIDEV_MODE1,       /* CPOL=0 CHPHA=1 */
	SPIDEV_MODE2,       /* CPOL=1 CHPHA=0 */
	SPIDEV_MODE3        /* CPOL=1 CHPHA=1 */
};
struct spi_dev_s {
	int unused;
};

//#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))
#define SPI_SELECT(d,id,s)
#else
#endif
