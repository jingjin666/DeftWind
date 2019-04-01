/****************************************************************************
 * config/imxrt1050-evk/src/imxrt_bringup.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/types.h>
#include <debug.h>

#ifdef CONFIG_IMXRT_USDHC
#include "imxrt_usdhc.h"
#include <nuttx/mm/gran.h>
#endif

#include "board_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

#if defined(CONFIG_FAT_DMAMEMORY)
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif
#endif

#define message(format, ...)    syslog(LOG_INFO, format, ##__VA_ARGS__)

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



/****************************************************************************
 * Private Functions
 ****************************************************************************/
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

#ifdef CONFIG_IMXRT_USDHC
    static int nsh_sdmmc_initialize(void)
    {
      struct sdio_dev_s *sdmmc;
      int ret = 0;
      /* Get an instance of the SDIO interface */
      dma_alloc_init();
      sdmmc = imxrt_usdhc_initialize(0);
      if (!sdmmc)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
        }
      else
        {
          /* Bind the SDIO interface to the MMC/SD driver */
          ret = mmcsd_slotinitialize(0, sdmmc);
          if (ret != OK)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
                     ret);
            }
        }
      return OK;
      }
#else
#  define nsh_sdmmc_initialize() (OK)
#endif

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

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  int ret;

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C1)
  imxrt_i2c_register(1);
#endif

#ifdef CONFIG_IMXRT_USDHC
    /* Initialize SDHC-base MMC/SD card support */
    syslog(LOG_INFO, "[boot] start sdmmc init\n");
    nsh_sdmmc_initialize();
#endif

  UNUSED(ret);
  return OK;
}
