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

#include "board_config.h"

/****************************************************************************
 * Name: nsh_sdmmc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USDHC
int uavrs_v2_usdhc_initialize(void)
{
	struct sdio_dev_s *sdmmc;
	int ret = 0;
	
	/* Get an instance of the SDIO interface */
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
			syslog(LOG_ERR, "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		}
	}
	return OK;
}
#endif
