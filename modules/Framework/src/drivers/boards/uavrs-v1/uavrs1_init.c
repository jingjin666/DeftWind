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
 * @file uavrs1_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>
#include <nuttx/board.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/
#define message(format, ...)    syslog(LOG_INFO, format, ##__VA_ARGS__)



#if defined(CONFIG_FAT_DMAMEMORY)
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif

static GRAN_HANDLE dma_allocator;

/*
 * The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */
static uint8_t g_dma_heap[8192] __attribute__((aligned(64)));
//static perf_counter_t g_dma_perf;

static void
dma_alloc_init(void)
{
	dma_allocator = gran_initialize(g_dma_heap,
					sizeof(g_dma_heap),
					7,  /* 128B granule - must be > alignment (XXX bug?) */
					6); /* 64B alignment */

	if (dma_allocator == NULL) {
            message("[boot] DMA allocator setup FAILED\n");
	} else {
	    message("[boot] DMA allocator setup OK\n");
            //g_dma_perf = perf_alloc(PC_COUNT, "DMA allocations");
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * DMA-aware allocator stubs for the FAT filesystem.
 */

__EXPORT void *fat_dma_alloc(size_t size);
__EXPORT void fat_dma_free(FAR void *memory, size_t size);

void *
fat_dma_alloc(size_t size)
{
	//perf_count(g_dma_perf);
	return gran_alloc(dma_allocator, size);
}

void
fat_dma_free(FAR void *memory, size_t size)
{
	gran_free(dma_allocator, memory, size);
}

#else

# define dma_alloc_init()

#endif

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
    message("stm32_boardinitialize\n");
    /* configure SPI interfaces */
    //stm32_spiinitialize();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;
static struct spi_dev_s *spi4;

static struct sdio_dev_s *sdio;

#include <math.h>

#ifdef CONFIG_LIB_BOARDCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{
    message("board_app_initialize\n");
    return nsh_archinitialize();
}

#endif /* CONFIG_LIB_BOARDCTL */


int nsh_archinitialize(void)
{
    message("nsh_archinitialize\n");
#if 0	
	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	stm32_configgpio(GPIO_ADC1_IN10);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN12);	/* BATT_VOLTAGE_SENS */
	//stm32_configgpio(GPIO_ADC1_IN14);	/* VBUS_ADC */
	stm32_configgpio(GPIO_ADC1_IN15);	/* PLANE_BATT_VOLTAGE_SNES */
	stm32_configgpio(GPIO_ADC1_IN8);	/* COPTER_BATT_VOLTAGE_SNES */
	stm32_configgpio(GPIO_ADC1_IN9);	/* STEERING_GEAR_BATT_VOLTAGE_SNES */

	/* configure the GPIO pins to outputs and keep them low */
	stm32_configgpio(GPIO_GPIO0_OUTPUT);
	stm32_configgpio(GPIO_GPIO1_OUTPUT);
	stm32_configgpio(GPIO_GPIO2_OUTPUT);
	stm32_configgpio(GPIO_GPIO3_OUTPUT);
	stm32_configgpio(GPIO_GPIO4_OUTPUT);
	stm32_configgpio(GPIO_GPIO5_OUTPUT);
	stm32_configgpio(GPIO_GPIO6_OUTPUT);
	stm32_configgpio(GPIO_GPIO7_OUTPUT);
	stm32_configgpio(GPIO_GPIO8_OUTPUT);
	stm32_configgpio(GPIO_GPIO9_OUTPUT);

	stm32_configgpio(GPIO_CAMERA_TRIGGER_OUTPUT);
	stm32_gpiowrite(GPIO_CAMERA_TRIGGER_OUTPUT, 1);
	
	stm32_configgpio(GPIO_CAMERA_FEEDBACK_INPUT);

	stm32_configgpio(GPIO_ADIS_DRDY);
	stm32_configgpio(GPIO_ADIS_RESET);
	stm32_gpiowrite(GPIO_ADIS_RESET, 1);

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure the DMA allocator */
	dma_alloc_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

	/* initial LED state */
	drv_led_start();
	led_off(LED_AMBER);
	/* Configure SPI-based devices */
	spi4 = stm32_spibus_initialize(4);
	if (!spi4) {
		message("[boot] FAILED to initialize SPI port 4\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI4 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi4, 10000000);
	SPI_SETBITS(spi4, 8);
	SPI_SETMODE(spi4, SPIDEV_MODE3);
	SPI_SELECT(spi4, PX4_SPIDEV_ADIS, false);
	up_udelay(20);
	
	/* Configure SPI-based devices */
	spi3 = stm32_spibus_initialize(3);
	if (!spi3) {
		message("[boot] FAILED to initialize SPI port 3\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI3 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi3, 10000000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);
	SPI_SELECT(spi3, PX4_SPIDEV_BARO_MS5611, false);
	SPI_SELECT(spi3, PX4_SPIDEV_MPU_9250, false);
	up_udelay(20);

	/* Get the SPI port for the FRAM */
	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		message("[boot] FAILED to initialize SPI port 2\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	 * and de-assert the known chip selects. */

	// XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, SPIDEVTYPE_FLASH, false);
#else
#ifdef CONFIG_MMCSD
    dma_alloc_init();

    /* First, get an instance of the SDIO interface */

    sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
    if (!sdio) {
    	message("[boot] Failed to initialize SDIO slot %d\n",
    		CONFIG_NSH_MMCSDSLOTNO);
    	return -ENODEV;
    } else {
        message("__%d sdio_initialize ok\n", __LINE__);
    }

    /* Now bind the SDIO interface to the MMC/SD driver */
    int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);
    if (ret != OK) {
    	message("[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
    	return ret;
    } else {
        message("__%d mmcsd_slotinitialize ok\n", __LINE__);
    }

    /* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
    sdio_mediachange(sdio, true);
    message("[boot] sdio configure complete\n");
#endif
#endif
	return OK;
}
#if 0
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

#if defined(CONFIG_FAT_DMAMEMORY)
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif

static GRAN_HANDLE dma_allocator;

/*
 * The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */
static uint8_t g_dma_heap[8192] __attribute__((aligned(64)));
static perf_counter_t g_dma_perf;

static void
dma_alloc_init(void)
{
	dma_allocator = gran_initialize(g_dma_heap,
					sizeof(g_dma_heap),
					7,  /* 128B granule - must be > alignment (XXX bug?) */
					6); /* 64B alignment */

	if (dma_allocator == NULL) {
		message("[boot] DMA allocator setup FAILED");

	} else {
		g_dma_perf = perf_alloc(PC_COUNT, "DMA allocations");
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * DMA-aware allocator stubs for the FAT filesystem.
 */

__EXPORT void *fat_dma_alloc(size_t size);
__EXPORT void fat_dma_free(FAR void *memory, size_t size);

void *
fat_dma_alloc(size_t size)
{
	perf_count(g_dma_perf);
	return gran_alloc(dma_allocator, size);
}

void
fat_dma_free(FAR void *memory, size_t size)
{
	gran_free(dma_allocator, memory, size);
}

#else

# define dma_alloc_init()

#endif

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure LEDs */
	//up_ledinit();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;
static struct spi_dev_s *spi4;

static struct sdio_dev_s *sdio;

#include <math.h>

__EXPORT int nsh_archinitialize(void)
{

	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	stm32_configgpio(GPIO_ADC1_IN10);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN12);	/* BATT_VOLTAGE_SENS */
	//stm32_configgpio(GPIO_ADC1_IN14);	/* VBUS_ADC */
	stm32_configgpio(GPIO_ADC1_IN15);	/* PLANE_BATT_VOLTAGE_SNES */
	stm32_configgpio(GPIO_ADC1_IN8);	/* COPTER_BATT_VOLTAGE_SNES */
	stm32_configgpio(GPIO_ADC1_IN9);	/* STEERING_GEAR_BATT_VOLTAGE_SNES */

	/* configure the GPIO pins to outputs and keep them low */
	stm32_configgpio(GPIO_GPIO0_OUTPUT);
	stm32_configgpio(GPIO_GPIO1_OUTPUT);
	stm32_configgpio(GPIO_GPIO2_OUTPUT);
	stm32_configgpio(GPIO_GPIO3_OUTPUT);
	stm32_configgpio(GPIO_GPIO4_OUTPUT);
	stm32_configgpio(GPIO_GPIO5_OUTPUT);
	stm32_configgpio(GPIO_GPIO6_OUTPUT);
	stm32_configgpio(GPIO_GPIO7_OUTPUT);
	stm32_configgpio(GPIO_GPIO8_OUTPUT);
	stm32_configgpio(GPIO_GPIO9_OUTPUT);

	stm32_configgpio(GPIO_CAMERA_TRIGGER_OUTPUT);
	stm32_gpiowrite(GPIO_CAMERA_TRIGGER_OUTPUT, 1);
	
	stm32_configgpio(GPIO_CAMERA_FEEDBACK_INPUT);

	stm32_configgpio(GPIO_ADIS_DRDY);
	stm32_configgpio(GPIO_ADIS_RESET);
	stm32_gpiowrite(GPIO_ADIS_RESET, 1);

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure the DMA allocator */
	dma_alloc_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

	/* initial LED state */
	drv_led_start();
	led_off(LED_AMBER);
	/* Configure SPI-based devices */
	spi4 = stm32_spibus_initialize(4);
	if (!spi4) {
		message("[boot] FAILED to initialize SPI port 4\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI4 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi4, 10000000);
	SPI_SETBITS(spi4, 8);
	SPI_SETMODE(spi4, SPIDEV_MODE3);
	SPI_SELECT(spi4, PX4_SPIDEV_ADIS, false);
	up_udelay(20);
	
	/* Configure SPI-based devices */
	spi3 = stm32_spibus_initialize(3);
	if (!spi3) {
		message("[boot] FAILED to initialize SPI port 3\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI3 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi3, 10000000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);
	SPI_SELECT(spi3, PX4_SPIDEV_BARO_MS5611, false);
	SPI_SELECT(spi3, PX4_SPIDEV_MPU_9250, false);
	up_udelay(20);

	/* Get the SPI port for the FRAM */
	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		message("[boot] FAILED to initialize SPI port 2\n");
		//up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	 * and de-assert the known chip selects. */

	// XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, SPIDEVTYPE_FLASH, false);

#ifdef CONFIG_MMCSD
	/* First, get an instance of the SDIO interface */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		message("[boot] Failed to initialize SDIO slot %d\n",
			CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		message("[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	/* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
	sdio_mediachange(sdio, true);

#endif

	return OK;
}
#endif
