/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "board_config.h"

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the imxrt1050-evk board.
 *
 ************************************************************************************/

void weak_function imxrt_spidev_initialize(void)
{
#ifdef CONFIG_IMXRT_LPSPI1
		(void)imxrt_config_gpio(GPIO_LPSPI1_CS_MPU9250);
		(void)imxrt_config_gpio(GPIO_LPSPI1_CS_BARO);
#endif

#ifdef CONFIG_IMXRT_LPSPI2
		(void)imxrt_config_gpio(GPIO_LPSPI2_CS);
#endif

#ifdef CONFIG_IMXRT_LPSPI3
		(void)imxrt_config_gpio(GPIO_LPSPI3_CS_ADIS16375BM);
#endif

#ifdef CONFIG_IMXRT_LPSPI4
		(void)imxrt_config_gpio(GPIO_LPSPI4_CS_FM25V05);
#endif

}

/****************************************************************************
 * Name:  imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *
 * Description:
 *   The external functions, imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including imxrt_lpspibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
__EXPORT void imxrt_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  	switch(devid) {
		case UAVRS_SPIDEV_BARO_MS5611:
			imxrt_gpio_write(GPIO_LPSPI1_CS_BARO, !selected);
            imxrt_gpio_write(GPIO_LPSPI1_CS_MPU9250, 1);
			break;
		case UAVRS_SPIDEV_MPU_9250:
			imxrt_gpio_write(GPIO_LPSPI1_CS_MPU9250, !selected);
            imxrt_gpio_write(GPIO_LPSPI1_CS_BARO, 1);
			break;
		default:
			break;
  }
}

__EXPORT uint8_t imxrt_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
    return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
__EXPORT void imxrt_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

    imxrt_gpio_write(GPIO_LPSPI2_CS, !selected);
}

__EXPORT uint8_t imxrt_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
    return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
__EXPORT void imxrt_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

    switch (devid) {
    case UAVRS_SPIDEV_ADIS:
        /* Making sure the other peripherals are not selected */
        imxrt_gpio_write(GPIO_LPSPI3_CS_ADIS16375BM, !selected);
        break;
    default:
        break;
  }
}

__EXPORT uint8_t imxrt_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
    return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
__EXPORT void imxrt_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

    imxrt_gpio_write(GPIO_LPSPI4_CS_FM25V05, !selected);
}

__EXPORT uint8_t imxrt_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
    return 0;
}
#endif

/****************************************************************************
 * Name: imxrt_lpspi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_IMXRT_LPSPI1
__EXPORT int imxrt_lpspi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
__EXPORT int imxrt_lpspi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
__EXPORT int imxrt_lpspi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
__EXPORT int imxrt_lpspi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 */
