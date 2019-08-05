/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

#ifndef __CONFIGS_UAVRS_V2_INCLUDE_BOARD_H
#define __CONFIGS_UAVRS_V2_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "chip/imxrt_pinmux.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        576Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 96
 *                        ARM_PODF_DIVISOR   = 2
 *                        576Mhz  = (24Mhz * 96/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 576 Mhz             = 576Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        576Mhz  = 576Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF, 1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 576Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       144Mhz = 576Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT / IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16Mhz = 144Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 576Mhz / IMXRT_SEMC_PODF_DIVIDER (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       72Mhz    = 576Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 *     FLEXCAN CLK = PLL3_CLK / 6 = (480Mhz / 6) = 80Mhz
 *                       IMXRT_FLEXCAN_PODF_DIVIDER = 4
 *                       20M =  80Mhz / 4
 */

#define BOARD_XTAL_FREQUENCY      24000000
#define IMXRT_PRE_PERIPH_CLK_SEL  CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL      CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT  96
#define IMXRT_ARM_PODF_DIVIDER    2
#define IMXRT_AHB_PODF_DIVIDER    1
#define IMXRT_IPG_PODF_DIVIDER    4
#define IMXRT_PERCLK_CLK_SEL      CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER 9
#define IMXRT_SEMC_PODF_DIVIDER   8
#define IMXRT_LPSPI_CLK_SELECT    CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER   8
#define IMXRT_USDHC1_CLK_SELECT    CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC1_PODF_DIVIDER 2
#define IMXRT_FLEXCAN_CLK_SELECT  CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80
#define IMXRT_FLEXCAN_PODF_DIVIDER 4

#define IMXRT_SYS_PLL_SELECT      CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define BOARD_CPU_FREQUENCY \
  (BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

#define BOARD_GPT_FREQUENCY \
	(BOARD_CPU_FREQUENCY / IMXRT_IPG_PODF_DIVIDER) / IMXRT_PERCLK_PODF_DIVIDER

/* SDIO *****************************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning for
 * specific boards, but has been checked by scope on the EVKB to make sure shapes
 * are square with minimal ringing.
 */

#define USDHC1_DATAX_IOMUX  (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | \
                             IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define USDHC1_CMD_IOMUX    (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | \
                             IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define USDHC1_CLK_IOMUX    (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | IOMUX_SPEED_MAX)
#define USDHC1_CD_IOMUX     (0)

#define PIN_USDHC1_D0       (GPIO_USDHC1_DATA0 | USDHC1_DATAX_IOMUX)
#define PIN_USDHC1_D1       (GPIO_USDHC1_DATA1 | USDHC1_DATAX_IOMUX)
#define PIN_USDHC1_D2       (GPIO_USDHC1_DATA2 | USDHC1_DATAX_IOMUX)
#define PIN_USDHC1_D3       (GPIO_USDHC1_DATA3 | USDHC1_DATAX_IOMUX)
#define PIN_USDHC1_DCLK     (GPIO_USDHC1_CLK   | USDHC1_CLK_IOMUX)
#define PIN_USDHC1_CMD      (GPIO_USDHC1_CMD   | USDHC1_CMD_IOMUX)
#define PIN_USDHC1_CD       (GPIO_USDHC1_CD_2  | USDHC1_CD_IOMUX)

/* 386 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* PIO Disambiguation ***************************************************************/

/* LPUARTs
 *
 * Virtual console port provided by OpenSDA:
 *
 *          UART1_TXD   GPIO_AD_B0_12  LPUART1_TX
 *          UART1_RXD   GPIO_AD_B0_13  LPUART1_RX
 *
 *   NOTE: There are no alternative pin configurations for LPUART1.
 *
 * Arduino RS-232 Shield:
 *
 *   J22 D0 UART_RX/D0  GPIO_AD_B1_07  LPUART3_RX
 *   J22 D1 UART_TX/D1  GPIO_AD_B1_06  LPUART3_TX
 */
#define GPIO_LPUART2_TX    GPIO_LPUART2_TX_1  /* GPIO_AD_B1_02----RTK-COM2-RX */
#define GPIO_LPUART2_RX    GPIO_LPUART2_RX_1  /* GPIO_AD_B1_03----RTK-COM2-TX */

#define GPIO_LPUART3_TX    GPIO_LPUART3_TX_1  /* GPIO_AD_B1_06----RTK-COM1-RX */
#define GPIO_LPUART3_RX    GPIO_LPUART3_RX_1  /* GPIO_AD_B1_07----RTK-COM1-TX */

#define GPIO_LPUART4_TX    GPIO_LPUART4_TX_1  /* GPIO_B1_00-------CBU-AUXUART2_TX */
#define GPIO_LPUART4_RX    GPIO_LPUART4_RX_1  /* GPIO_B1_01-------CBU-AUXUART2_RX */

#define GPIO_LPUART5_TX    GPIO_LPUART5_TX_1  /* GPIO_B1_12-------CBU-AUXUART1_TX */
#define GPIO_LPUART5_RX    GPIO_LPUART5_RX_1  /* GPIO_B1_13-------CBU-AUXUART1_RX */

#define GPIO_LPUART6_TX    GPIO_LPUART6_TX_2  /* GPIO_EMC_25------P900-DAT-RXD */
#define GPIO_LPUART6_RX    GPIO_LPUART6_RX_2  /* GPIO_EMC_26------P900-DAT-TXD */

#define GPIO_LPUART7_TX    GPIO_LPUART7_TX_1  /* GPIO_EMC_31------CBU-SBUS_OUTPUT */
#define GPIO_LPUART7_RX    GPIO_LPUART7_RX_1  /* GPIO_EMC_32------CBU-SBUS_INPUT */

#define GPIO_LPUART8_TX    GPIO_LPUART8_TX_1  /* GPIO_AD_B1_10----CBU-AUXGPS_TX */
#define GPIO_LPUART8_RX    GPIO_LPUART8_RX_1  /* GPIO_AD_B1_11----CBU-AUXGPS_RX */

/* LPI2Cs
 *
 */

#define GPIO_LPI2C1_SDA   GPIO_LPI2C1_SDA_2  /* GPIO_AD_B1_01-----AirSpeed */
#define GPIO_LPI2C1_SCL   GPIO_LPI2C1_SCL_2  /* GPIO_AD_B1_00-----AirSpeed */

#define GPIO_LPI2C2_SDA   GPIO_LPI2C2_SDA_2  /* GPIO_AD_B1_01-----NotConnect */
#define GPIO_LPI2C2_SCL   GPIO_LPI2C2_SCL_2  /* GPIO_AD_B1_00-----NotConnect */

#define GPIO_LPI2C3_SDA   GPIO_LPI2C3_SDA_2  /* GPIO_EMC_21-------NotConnect */
#define GPIO_LPI2C3_SCL   GPIO_LPI2C3_SCL_2  /* GPIO_EMC_22-------NotConnect */

#define GPIO_LPI2C4_SDA   GPIO_LPI2C4_SDA_2  /* GPIO_EMC_11-------NotConnect */
#define GPIO_LPI2C4_SCL   GPIO_LPI2C4_SCL_2  /* GPIO_EMC_12-------NotConnect */

/* LPSPI
 *
 */

#define GPIO_LPSPI1_SCK   GPIO_LPSPI1_SCK_1  /* GPIO_EMC_27------MS5611/MPU-9250 */
#define GPIO_LPSPI1_MOSI  GPIO_LPSPI1_SDO_1  /* GPIO_EMC_28------MS5611/MPU-9250 */
#define GPIO_LPSPI1_MISO  GPIO_LPSPI1_SDI_1  /* GPIO_EMC_29------MS5611/MPU-9250 */

#define GPIO_LPSPI2_SCK   GPIO_LPSPI2_SCK_1  /* GPIO_EMC_00------NotConnect */
#define GPIO_LPSPI2_MOSI  GPIO_LPSPI2_SDO_1  /* GPIO_EMC_02------NotConnect */
#define GPIO_LPSPI2_MISO  GPIO_LPSPI2_SDI_1  /* GPIO_EMC_03------NotConnect */

#define GPIO_LPSPI3_SCK   GPIO_LPSPI3_SCK_2  /* GPIO_AD_B0_00----ADIS16375BM */
#define GPIO_LPSPI3_MOSI  GPIO_LPSPI3_SDO_2  /* GPIO_AD_B0_01----ADIS16375BM */
#define GPIO_LPSPI3_MISO  GPIO_LPSPI3_SDI_2  /* GPIO_AD_B0_02----ADIS16375BM */

#define GPIO_LPSPI4_SCK   GPIO_LPSPI4_SCK_1  /* GPIO_B1_07-------FM25V05-GTR */
#define GPIO_LPSPI4_MOSI  GPIO_LPSPI4_SDO_1  /* GPIO_B1_06-------FM25V05-GTR */
#define GPIO_LPSPI4_MISO  GPIO_LPSPI4_SDI_1  /* GPIO_B1_05-------FM25V05-GTR */

/* FLEXCAN
 *
 */
#define GPIO_CAN1_RX	GPIO_FLEXCAN1_RX_4
#define GPIO_CAN1_TX	GPIO_FLEXCAN1_TX_4
#define GPIO_CAN2_RX	GPIO_FLEXCAN2_RX_4
#define GPIO_CAN2_TX	GPIO_FLEXCAN2_TX_4

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_UAVRS_V2_INCLUDE_BOARD_H */
