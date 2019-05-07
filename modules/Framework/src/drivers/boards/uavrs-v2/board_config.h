/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

#ifndef __CONFIGS_BOARD_CONFIG
#define __CONFIGS_BOARD_CONFIG

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

/* i.MX RT 1050 GPIO Pin Definitions ****************************************/

/* LEDs
 *
 * There are four LED status indicators located on the EVK Board.  The
 * functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.  It connects to
 * GPIO_AD_B0_09 which is shared with JTAG_TDI and ENET_RST.  This pin
 * must be configured as ALT5, GPIO1_IO09
 */

#define IOMUX_LED       (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                         IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                         IOMUX_SLEW_SLOW)
#define GPIO_LED        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | \
                         GPIO_PIN9 | IOMUX_LED)

/* Buttons
 *
 * The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 *
 */
#define IOMUX_SW8       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_SW8        (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                         GPIO_PORT5 | GPIO_PIN0 | IOMUX_SW8)

/* Ethernet Interrupt: GPIOAD_B0_10
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define IOMUX_ENET_INT  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K)
#define GPIO_ENET_INT   (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                         GPIO_PORT1 | GPIO_PIN10 | IOMUX_ENET_INT)
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO1_10

/* Ethernet Reset:  GPIOAD_B0_09
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define IOMUX_ENET_RST  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                          GPIO_PORT1 | GPIO_PIN9 | IOMUX_ENET_RST)

/* LPSPI1 CS:  GPIO_SD_B0_01 */

#define IOMUX_LPSPI1_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI1_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT3 | GPIO_PIN13 | IOMUX_LPSPI1_CS)

#define IOMUX_MMCSD_EN  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_MMCSD_EN   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT3 | GPIO_PIN2 | IOMUX_MMCSD_EN)

/* LPSPI3 CS:  GPIO_AD_B0_03 */

#define IOMUX_LPSPI3_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI3_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT1 | GPIO_PIN3 | IOMUX_LPSPI3_CS)

/* High-resolution timer */

#define HRT_TIMER               1  /* use GPT1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

//#define HRT_PPM_CHANNEL         /* GPT1_CAPTURE2 */  2  /* use capture/compare channel 2 */
//#define GPIO_PPM_IN             /* GPT1_CAPTURE2 */ GPIO_GPT1_CAPTURE2

/* ADC */

#define ADC_IOMUX (IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n, p)             (GPIO_PORT1 | GPIO_PIN##p | ADC_IOMUX)
#define DP_ADC_GPIO  \
	/* COPTER_BATTERY_VOLTAGE          GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_GPIO(3,  14),  \
	/* PLANE_BATTERY_VOLTAGE           GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_GPIO(10,  21),  \
	/* STEERING_BATTERY_VOLTAGE        GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9, 20)
#define ADC_COPTER_BATTERY_VOLTAGE_CHANNEL          /* GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_CH(3)
#define ADC_PLANE_BATTERY_VOLTAGE_CHANNEL           /* GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_CH(10)
#define ADC_STEERING_GEAR_BATTERY_VOLTAGE_CHANNEL   /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_CH(9)
#define ADC_CHANNELS \
	((1 << ADC_COPTER_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_PLANE_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_STEERING_GEAR_BATTERY_VOLTAGE_CHANNEL))

/* USB OTG VBUS 
 * GPIO_AD_B0_15
 * ADC1_IN4
 * GPIO1_IO15
 */
#define IOMUX_USB_OTG_VBUS       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_USB_OTG_VBUS        (GPIO_INPUT | \
                         GPIO_PORT1 | GPIO_PIN15 | IOMUX_USB_OTG_VBUS)

/* By Providing BOARD_ADC_USB_CONNECTED (using the dp_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED dp_arch_gpioread(GPIO_USB_OTG_VBUS)

/* The list of GPIO that will be initialized */

#define DP_GPIO_INIT_LIST { \
		DP_ADC_GPIO,            \
}



#define UAVRS_SPI_BUS_ADIS	4

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define UAVRS_SPIDEV_ADIS		1


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uavrs_v2_usdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USDHC
int uavrs_v2_usdhc_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the i.MXRT1050 EVK.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
void imxrt_spidev_initialize(void);
#endif

int imxrt_usb_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_BOARD_CONFIG */
