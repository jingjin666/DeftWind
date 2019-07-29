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

//#define CONFIG_IMXRT1052_HYPER_FLASH 1

#define CONFIG_IMXRT1052_QSPI_FLASH

//#define CONFIG_IMXRT1064_QSPI_FLASH 1

//#define CONFIG_IMXRT_SEMC_INIT_DONE 1

#define UDID_START		0x1FFF7A10

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM Capture
 *
 * 0  PWM Capture inputs are not supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  0

/* PWM
 *
 * 16  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1  : GPIO_EMC_06   GPIO4_IO6       FLEXPWM2_PWMA00
 * FMU_CH2  : GPIO_EMC_08   GPIO4_IO8       FLEXPWM2_PWMA01 //camera use
 * FMU_CH2  : GPIO_EMC_10   GPIO4_IO10      FLEXPWM2_PWMA02
 * FMU_CH3  : GPIO_B1_02    GPIO2_IO18      FLEXPWM2_PWMA03
 * FMU_CH4  : GPIO_EMC_07   GPIO4_IO7       FLEXPWM2_PWMB00
 * FMU_CH5  : GPIO_EMC_09   GPIO4_IO9       FLEXPWM2_PWMB01
 * FMU_CH6  : GPIO_B1_03    GPIO2_IO19      FLEXPWM2_PWMB03
 *
 * FMU_CH7  : GPIO_AD_B1_08 GPIO1_IO24      FLEXPWM4_PWMA00
 * FMU_CH8  : GPIO_AD_B1_09 GPIO1_IO25      FLEXPWM4_PWMA01
 * FMU_CH9  : GPIO_EMC_04   GPIO4_IO4       FLEXPWM4_PWMA02
 * FMU_CH10 : GPIO_B1_15    GPIO2_IO31      FLEXPWM4_PWMA03
 *
 * FMU_CH11 : GPIO_EMC_23   GPIO4_IO23      FLEXPWM1_PWMA00
 * FMU_CH12 : GPIO_EMC_24   GPIO4_IO24      FLEXPWM1_PWMB00
 * FMU_CH13 : GPIO_EMC_39   GPIO3_IO25      FLEXPWM1_PWMB03
 *
 * FMU_CH14 : GPIO_EMC_33   GPIO3_IO19      FLEXPWM3_PWMA02
 * FMU_CH15 : GPIO_EMC_34   GPIO3_IO20      FLEXPWM3_PWMB02
 *
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  15

#define PWM_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define PIN_FLEXPWM2_PWMA00     (PWM_IOMUX | GPIO_FLEXPWM2_PWMA00_1)
//#define PIN_FLEXPWM2_PWMA01     (PWM_IOMUX | GPIO_FLEXPWM2_PWMA01_1)  //camera use
#define PIN_FLEXPWM2_PWMA02     (PWM_IOMUX | GPIO_FLEXPWM2_PWMA02_1)
#define PIN_FLEXPWM2_PWMA03     (PWM_IOMUX | GPIO_FLEXPWM2_PWMA03_5)
#define PIN_FLEXPWM2_PWMB00     (PWM_IOMUX | GPIO_FLEXPWM2_PWMB00_1)
#define PIN_FLEXPWM2_PWMB01     (PWM_IOMUX | GPIO_FLEXPWM2_PWMB01_1)
#define PIN_FLEXPWM2_PWMB03     (PWM_IOMUX | GPIO_FLEXPWM2_PWMB03_4)

#define PIN_FLEXPWM4_PWMA00     (PWM_IOMUX | GPIO_FLEXPWM4_PWMA00_1)
#define PIN_FLEXPWM4_PWMA01     (PWM_IOMUX | GPIO_FLEXPWM4_PWMA01_1)
#define PIN_FLEXPWM4_PWMA02     (PWM_IOMUX | GPIO_FLEXPWM4_PWMA02_2)
#define PIN_FLEXPWM4_PWMA03     (PWM_IOMUX | GPIO_FLEXPWM4_PWMA03_1)

#define PIN_FLEXPWM1_PWMA00     (PWM_IOMUX | GPIO_FLEXPWM1_PWMA00_1)
#define PIN_FLEXPWM1_PWMB00     (PWM_IOMUX | GPIO_FLEXPWM1_PWMB00_1)
#define PIN_FLEXPWM1_PWMB03     (PWM_IOMUX | GPIO_FLEXPWM1_PWMB03_2)

#define PIN_FLEXPWM3_PWMA02     (PWM_IOMUX | GPIO_FLEXPWM3_PWMA02)
#define PIN_FLEXPWM3_PWMB02     (PWM_IOMUX | GPIO_FLEXPWM3_PWMB02)

/* User GPIOs
 *
 * GPIO-
 * Define as GPIO input / GPIO outputs
 */

#define FMU_INPUT_IOMUX (IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|FMU_INPUT_IOMUX))

#define GPIO_GPIO0_INPUT        /* P4:6   PWM2 A0 FMU1  */ (GPIO_PORT4 | GPIO_PIN6  | GPIO_INPUT | FMU_INPUT_IOMUX)
//#define GPIO_GPIO1_INPUT        /* P4:8   PWM2 A1 FMU2  */ (GPIO_PORT4 | GPIO_PIN8  | GPIO_INPUT | FMU_INPUT_IOMUX)       //camera use
#define GPIO_GPIO1_INPUT        /* P4:10  PWM2 A2 FMU2  */ (GPIO_PORT4 | GPIO_PIN10 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO2_INPUT        /* P2:18  PWM2 A3 FMU3  */ (GPIO_PORT2 | GPIO_PIN18 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO3_INPUT        /* P4:7   PWM2 B0 FMU4  */ (GPIO_PORT4 | GPIO_PIN7  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO4_INPUT        /* P4:9   PWM2 B1 FMU5  */ (GPIO_PORT4 | GPIO_PIN9  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO5_INPUT        /* P2:19  PWM2 B3 FMU6  */ (GPIO_PORT2 | GPIO_PIN19 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO6_INPUT        /* P1:24  PWM4 A0 FMU7  */ (GPIO_PORT1 | GPIO_PIN24 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO7_INPUT        /* P1:25  PWM4 A1 FMU8  */ (GPIO_PORT1 | GPIO_PIN25 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO8_INPUT        /* P4:4   PWM4 A2 FMU9  */ (GPIO_PORT4 | GPIO_PIN4  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO9_INPUT        /* P2:31  PWM4 A3 FMU10 */ (GPIO_PORT2 | GPIO_PIN31 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO10_INPUT       /* P4:23  PWM1 A0 FMU11 */ (GPIO_PORT4 | GPIO_PIN23 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO11_INPUT       /* P4:24  PWM1 B0 FMU12 */ (GPIO_PORT4 | GPIO_PIN24 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO12_INPUT       /* P3:25  PWM1 B3 FMU13 */ (GPIO_PORT3 | GPIO_PIN25 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO13_INPUT       /* P3:19  PWM3 A2 FMU14 */ (GPIO_PORT3 | GPIO_PIN19 | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_GPIO14_INPUT       /* P3:20  PWM3 B2 FMU15 */ (GPIO_PORT3 | GPIO_PIN20 | GPIO_INPUT | FMU_INPUT_IOMUX)

#define FMU_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|FMU_OUTPUT_IOMUX))

#define GPIO_GPIO0_OUTPUT        /* P4:6   PWM2 A0 FMU1  */ (GPIO_PORT4 | GPIO_PIN6  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
//#define GPIO_GPIO1_OUTPUT        /* P4:8   PWM2 A1 FMU2  */ (GPIO_PORT4 | GPIO_PIN8  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)    //camera use
#define GPIO_GPIO1_OUTPUT        /* P4:10  PWM2 A2 FMU2  */ (GPIO_PORT4 | GPIO_PIN10 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO2_OUTPUT        /* P2:18  PWM2 A3 FMU3  */ (GPIO_PORT2 | GPIO_PIN18 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO3_OUTPUT        /* P4:7   PWM2 B0 FMU4  */ (GPIO_PORT4 | GPIO_PIN7  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO4_OUTPUT        /* P4:9   PWM2 B1 FMU5  */ (GPIO_PORT4 | GPIO_PIN9  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO5_OUTPUT        /* P2:19  PWM2 B3 FMU6  */ (GPIO_PORT2 | GPIO_PIN19 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO6_OUTPUT        /* P1:24  PWM4 A0 FMU7  */ (GPIO_PORT1 | GPIO_PIN24 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO7_OUTPUT        /* P1:25  PWM4 A1 FMU8  */ (GPIO_PORT1 | GPIO_PIN25 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO8_OUTPUT        /* P4:4   PWM4 A2 FMU9  */ (GPIO_PORT4 | GPIO_PIN4  | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO9_OUTPUT        /* P2:31  PWM4 A3 FMU10 */ (GPIO_PORT2 | GPIO_PIN31 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO10_OUTPUT       /* P4:23  PWM1 A0 FMU11 */ (GPIO_PORT4 | GPIO_PIN23 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO11_OUTPUT       /* P4:24  PWM1 B0 FMU12 */ (GPIO_PORT4 | GPIO_PIN24 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO12_OUTPUT       /* P3:25  PWM1 B3 FMU13 */ (GPIO_PORT3 | GPIO_PIN25 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO13_OUTPUT       /* P3:19  PWM3 A2 FMU14 */ (GPIO_PORT3 | GPIO_PIN19 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)
#define GPIO_GPIO14_OUTPUT       /* P3:20  PWM3 B2 FMU15 */ (GPIO_PORT3 | GPIO_PIN20 | GPIO_OUTPUT | FMU_OUTPUT_IOMUX)

/* 5V Camer trigger signal */
#define GPIO_CAMERA_TRIGGER_INPUT    /* FLEXPWM2_PWMA01 GPIO_EMC_08 P4:8 ALT5*/ (GPIO_PORT4 | GPIO_PIN8  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_CAMERA_TRIGGER_OUTPUT   /* FLEXPWM2_PWMA01 GPIO_EMC_08 P4:8 ALT5*/ (GPIO_PORT4 | GPIO_PIN8  | GPIO_OUTPUT| FMU_OUTPUT_IOMUX)

/* 3.3V Camer feedback signal */
#define GPIO_CAMERA_FEEDBACK_INPUT    /* TMR2_TIMER0 GPIO_EMC_19 P4:19  ALT5*/ (GPIO_PORT4 | GPIO_PIN19  | GPIO_INPUT | FMU_INPUT_IOMUX)
#define GPIO_CAMERA_FEEDBACK_OUTPUT   /* TMR2_TIMER0 GPIO_EMC_19 P4:19  ALT5*/ (GPIO_PORT4 | GPIO_PIN19  | GPIO_OUTPUT| FMU_OUTPUT_IOMUX)

/* Monitoring GPIOs */
#define GENERAL_INPUT_IOMUX  (IOMUX_PULL_UP_100K| _IOMUX_PULL_ENABLE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE    | IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)

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

#define IOMUX_PGOOD       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
/* INS_PGOOD
 * GPIO_SD_B1_01
 * GPIO3_IO01
 */                         
#define GPIO_INS_PGOOD        (GPIO_INPUT | GPIO_PORT3 | GPIO_PIN1 | IOMUX_PGOOD)
#define BOARD_INS_PGOOD     dp_arch_gpioread(GPIO_INS_PGOOD)

/* P900_PGOOD
 * GPIO_SD_B1_04
 * GPIO3_IO04
 */
#define GPIO_P900_PGOOD        (GPIO_INPUT | GPIO_PORT3 | GPIO_PIN4 | IOMUX_PGOOD)
#define BOARD_P900_PGOOD     dp_arch_gpioread(GPIO_P900_PGOOD)

/* RTK_PGOOD
 * GPIO_EMC_35
 * GPIO3_IO21
 */
#define GPIO_RTK_PGOOD        (GPIO_INPUT | GPIO_PORT3 | GPIO_PIN21 | IOMUX_PGOOD)
#define BOARD_RTK_PGOOD     dp_arch_gpioread(GPIO_RTK_PGOOD)

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
		GPIO_INS_PGOOD,         \
		GPIO_P900_PGOOD,        \
		GPIO_RTK_PGOOD,         \
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

/* RC Chanle SBUS [UART7] */
#define RC_SERIAL_PORT      "/dev/ttyS6"

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
