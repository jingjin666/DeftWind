/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include <debug.h>

#ifdef CONFIG_IMXRT_USDHC
#include "imxrt_usdhc.h"
#include <nuttx/mm/gran.h>
#endif

#include "imxrt_start.h"
#include "board_config.h"
#include "dp_micro_hal.h"
#include <systemlib/dp_macros.h>
#include <systemlib/cpuload.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* Debug ********************************************************************/
#ifdef CONFIG_CPP_HAVE_VARARGS
#define message(format, ...)    syslog(LOG_INFO, format, ##__VA_ARGS__)
#endif

#if defined(CONFIG_FAT_DMAMEMORY)
#if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#error microSD DMA support requires CONFIG_GRAN
#endif

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
#define dma_alloc_init()
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

static int nsh_archinitialize(void)
{
	int ret;
	
	hrt_init();

	/* configure the DMA allocator */
	dma_alloc_init();

    /* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
    cpuload_initialize_once();
#endif

	/* initial LED state */
	drv_led_start();
	led_on(LED_WORKSTATUS);

#ifdef CONFIG_IMXRT_USDHC
	/* Initialize SDHC-base MMC/SD card support */
	ret = uavrs_v2_usdhc_initialize();
	if (ret < 0)
	{
		syslog(LOG_ERR, "Failed to initialize sdmmc Driver: %d\n", ret);
		return ret;
	}
#endif
	return OK;
}


/************************************************************************************
 * Name: board_gpio_init
 *
 * Description:
 *   Board may provide a list of GPI pins to get initialized
 *
 *  list    - A list of GPIO pins to be initialized
 *  count   - Size of the list
 *
 * return  - Nothing
  ************************************************************************************/


static void board_gpio_init(const uint32_t list[], int count)
{
	for (int gpio = 0; gpio < count; gpio++) {
		if (list[gpio] != 0) {
			dp_arch_configgpio(list[gpio]);
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

__EXPORT void imxrt_boardinitialize(void)
{
	led_init();

	/* configure pins */
	const uint32_t gpio[] = DP_GPIO_INIT_LIST;
	board_gpio_init(gpio, arraySize(gpio));

	imxrt_spidev_initialize();

	imxrt_usb_initialize();
    
	fmurt1052_timer_initialize();
}

#ifdef CONFIG_LIB_BOARDCTL


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
	return nsh_archinitialize();
}

#endif /* CONFIG_LIB_BOARDCTL */
