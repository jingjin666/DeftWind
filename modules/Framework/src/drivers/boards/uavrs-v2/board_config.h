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


#define UDID_START		0x1FFF7A10

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM Capture
 *
 * 3  PWM Capture inputs are not supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  0

/* PWM
 *
 * 8  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : GPIO_B0_06    GPIO2 Pin 6  FLEXPWM2_PWMA0
 * FMU_CH2 : GPIO_EMC_08   GPIO4 Pin 8  FLEXPWM2_PWMA1
 * FMU_CH3 : GPIO_EMC_10   GPIO4 Pin 10 FLEXPWM2_PWMA2
 * FMU_CH4 : GPIO_AD_B0_09 GPIO1 Pin 9  FLEXPWM2_PWMA3
 * FMU_CH5 : GPIO_EMC_33   GPIO3 Pin 19 FLEXPWM3_PWMA2
 * FMU_CH6 : GPIO_EMC_30   GPIO4 Pin 30 FLEXPWM3_PWMB0
 * FMU_CH7 : GPIO_EMC_04   GPIO4 Pin 4  FLEXPWM4_PWMA2
 * FMU_CH8 : GPIO_EMC_01   GPIO4 Pin 1  FLEXPWM4_PWMB0
 *
 */
#define PWM_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define PIN_FLEXPWM2_PWMA00  /* P2:6  PWM2 A0 FMU1 */ (PWM_IOMUX | GPIO_FLEXPWM2_PWMA00_2)
#define PIN_FLEXPWM2_PWMA01  /* P4:8  PWM2 A1 FMU2 */ (PWM_IOMUX | GPIO_FLEXPWM2_PWMA01_1)
#define PIN_FLEXPWM2_PWMA02  /* P4:10 PWM2 A2 FMU3 */ (PWM_IOMUX | GPIO_FLEXPWM2_PWMA02_1)
#define PIN_FLEXPWM2_PWMA03  /* P1:9  PWM2 A3 FMU4 */ (PWM_IOMUX | GPIO_FLEXPWM2_PWMA03_2)
#define PIN_FLEXPWM3_PWMA02  /* P3:19 PWM3 A2 FMU5 */ (PWM_IOMUX | GPIO_FLEXPWM3_PWMA02)
#define PIN_FLEXPWM3_PWMB00  /* P4:30 PWM3 B0 FMU6 */ (PWM_IOMUX | GPIO_FLEXPWM3_PWMB00)
#define PIN_FLEXPWM4_PWMA02  /* P4:4  PWM4 A2 FMU7 */ (PWM_IOMUX | GPIO_FLEXPWM4_PWMA02_2)
#define PIN_FLEXPWM4_PWMB00  /* P4:1  PWM4 B0 FMU8 */ (PWM_IOMUX | GPIO_FLEXPWM4_PWMB00)

#define DIRECT_PWM_OUTPUT_CHANNELS  8

/* User GPIOs
 *
 * GPIO-
 * Define as GPIO input / GPIO outputs
 */

#define FMU_INPUT_IOMUX (IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|FMU_INPUT_IOMUX))

#define GPIO_GPIO0_INPUT        /* P2:6  PWM2 A0 FMU1 */ (GPIO_PORT2 | GPIO_PIN6  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO1_INPUT        /* P4:8  PWM2 A1 FMU2 */ (GPIO_PORT4 | GPIO_PIN8  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO2_INPUT        /* P4:10 PWM2 A2 FMU3 */ (GPIO_PORT4 | GPIO_PIN10 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO3_INPUT        /* P1:9  PWM2 A3 FMU4 */ (GPIO_PORT1 | GPIO_PIN9  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO4_INPUT        /* P3:19 PWM3 A2 FMU5 */ (GPIO_PORT3 | GPIO_PIN19 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO5_INPUT        /* P4:30 PWM3 B0 FMU6 */ (GPIO_PORT4 | GPIO_PIN30 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO6_INPUT        /* P4:4  PWM4 A2 FMU7 */ (GPIO_PORT4 | GPIO_PIN4  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO7_INPUT        /* P4:1  PWM4 B0 FMU8 */ (GPIO_PORT1 | GPIO_PIN1  | GPIO_INPUT | FMU_INPUT_IOMUX)

#define FMU_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|FMU_OUTPUT_IOMUX))

#define GPIO_GPIO0_OUTPUT        /* P2:6  PWM2 A0 FMU1 */ (GPIO_PORT2 | GPIO_PIN6  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO1_OUTPUT        /* P4:8  PWM2 A1 FMU2 */ (GPIO_PORT4 | GPIO_PIN8  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO2_OUTPUT        /* P4:10 PWM2 A2 FMU3 */ (GPIO_PORT4 | GPIO_PIN10 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO3_OUTPUT        /* P1:9  PWM2 A3 FMU4 */ (GPIO_PORT1 | GPIO_PIN9  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO4_OUTPUT        /* P3:19 PWM3 A2 FMU5 */ (GPIO_PORT3 | GPIO_PIN19 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO5_OUTPUT        /* P4:30 PWM3 B0 FMU6 */ (GPIO_PORT4 | GPIO_PIN30 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO6_OUTPUT        /* P4:4  PWM4 A2 FMU7 */ (GPIO_PORT4 | GPIO_PIN4  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO7_OUTPUT        /* P4:1  PWM4 B0 FMU8 */ (GPIO_PORT1 | GPIO_PIN1  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)

/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)


#define IOMUX_PMIC_STBY_REQ     (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                                 IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                                 IOMUX_SLEW_SLOW)
#define GPIO_PMIC_STBY_REQ      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT5 | \
                                 GPIO_PIN2 | IOMUX_PMIC_STBY_REQ)

#define IOMUX_LED       (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                         IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                         IOMUX_SLEW_SLOW)
#define GPIO_LED        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT3 | \
                         GPIO_PIN0 | IOMUX_LED)

#define IOMUX_ADIS_DRDY       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_ADIS_DRDY      (GPIO_PORT4 | GPIO_PIN13  | GPIO_INPUT | IOMUX_ADIS_DRDY)

#define IOMUX_ADIS_RESET       (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                         IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                         IOMUX_SLEW_SLOW)
#define GPIO_ADIS_RESET     (GPIO_PORT2 | GPIO_PIN3  | GPIO_OUTPUT | IOMUX_ADIS_RESET)

/* LPSPI1 CS:  MPU9250--GPIO_EMC_30  BARO--GPIO_EMC_14 */
#define IOMUX_LPSPI1_CS             (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI1_CS_MPU9250      (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN30 | IOMUX_LPSPI1_CS)
#define GPIO_LPSPI1_CS_BARO         (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN14 | IOMUX_LPSPI1_CS)

/* LPSPI2 CS:  GPIO_EMC_01 */
#define IOMUX_LPSPI2_CS             (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI2_CS              (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN1 | IOMUX_LPSPI2_CS)

/* LPSPI3 CS:  ADIS16375BM--GPIO_AD_B0_03 */
#define IOMUX_LPSPI3_CS             (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI3_CS_ADIS16375BM  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT1 | GPIO_PIN3 | IOMUX_LPSPI3_CS)

/* LPSPI4 CS:  FM25V05--GPIO_B1_04 */
#define IOMUX_LPSPI4_CS             (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI4_CS_FM25V05      (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT2 | GPIO_PIN20 | IOMUX_LPSPI4_CS)

/* Backup IMU */
#define UAVRS_SPI_BUS_MPU_9250		1
/* Baro MS5611 */
#define UAVRS_SPI_BUS_BARO_MS5611	1
/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define UAVRS_SPIDEV_BARO_MS5611	1
#define UAVRS_SPIDEV_MPU_9250		2

/* Main IMU */
#define UAVRS_SPI_BUS_ADIS	3
/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3 */
#define UAVRS_SPIDEV_ADIS   1

/* Mtd Storage Device */
#define UAVRS_SPI_BUS_RAMTRON	4


/* MMCSD EN */
#define IOMUX_MMCSD_EN  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                       _IOMUX_PULL_ENABLE)
#define GPIO_MMCSD_EN   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                        GPIO_PORT3 | GPIO_PIN2 | IOMUX_MMCSD_EN)


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
        GPIO_PMIC_STBY_REQ,     \
		DP_ADC_GPIO,            \
}

#define DP_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO7_INPUT, \
		GPIO_GPIO6_INPUT, \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT, \
		GPIO_GPIO0_INPUT, \
}


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

void fmurt1052_timer_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_BOARD_CONFIG */
