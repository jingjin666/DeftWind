/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * UAVRSv1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <dp_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

#define UDID_START		0x1FFF7A10

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
/* Work Status Led */
#define GPIO_LED1			(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)

/* External interrupts */

/* Data ready pins on */
#define GPIO_ADIS_DRDY 				(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN13)

#define GPIO_ADIS_RESET 			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

/* SPI4 off */
#define GPIO_SPI4_SCK_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI4_MISO_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN5)
#define GPIO_SPI4_MOSI_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN6)

/* SPI4 chip selects off */
#define GPIO_SPI_CS_ADIS_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN4)

/* SPI4 chip selects */
#define GPIO_SPI_CS_ADIS			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

/* SPI3 off */
#define GPIO_SPI3_SCK_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN3)
#define GPIO_SPI3_MISO_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI3_MOSI_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN5)

/* SPI3 chip selects off */
#define GPIO_SPI_CS_BARO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN4)
#define GPIO_SPI_CS_MPU_OFF			(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN3)

/* SPI3 chip selects */
#define GPIO_SPI_CS_BARO			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_SPI_CS_MPU				(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)

/* SPI2 chip selects */
#define GPIO_SPI_CS_FRAM	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)

/* Main IMU */
#define PX4_SPI_BUS_ADIS	4

/* Backup IMU and Baro */
#define PX4_SPI_BUS_BARO_MS5611			3
#define PX4_SPI_BUS_MPU_9250			3

/* Mtd Storage Device */
#define PX4_SPI_BUS_RAMTRON	2

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define PX4_SPIDEV_ADIS		1

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3 */
#define PX4_SPIDEV_BARO_MS5611	1
#define PX4_SPIDEV_MPU_9250		2

/* I2C1 busses */
#define PX4_I2C_BUS_ONBOARD			1

/* I2C2 busses */
#define PX4_I2C_BUS_EXPANSION		2

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 4) | (1 << 8) | (1 << 9) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_5V_RAIL_SENSE		4
#define ADC_COPTER_BATTERY_VOLTAGE_CHANNEL	8
#define ADC_STEERING_GEAR_BATTERY_VOLTAGE_CHANNEL	9
#define ADC_PLANE_BATTERY_VOLTAGE_CHANNEL	15

/* User GPIOs
 *
 * GPIO0-7 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO1_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO2_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO3_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO4_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN7)
#define GPIO_GPIO8_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO9_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)

#define GPIO_GPIO0_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO1_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO2_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO3_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO4_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO5_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO6_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_GPIO7_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_GPIO8_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO9_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)

/* 5V Camer trigger signal */
#define GPIO_CAMERA_TRIGGER_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN6)
#define GPIO_CAMERA_TRIGGER_OUTPUT    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN6)

/* 3.3V Camer feedback signal */
#define GPIO_CAMERA_FEEDBACK_INPUT    (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN15)
#define GPIO_CAMERA_FEEDBACK_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* PWMs
 *
 * 12 PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE9  : TIM1_CH1
 * CH2 : PE11 : TIM1_CH2
 * CH3 : PE13 : TIM1_CH3
 * CH4 : PE14 : TIM1_CH4
 * CH5 : PD12 : TIM4_CH1
 * CH6 : PD13 : TIM4_CH2
 * CH7 : PD14 : TIM4_CH3
 * CH8 : PD15 : TIM4_CH4
 * CH9 : PA2 :   TIM2_CH3
 * CH10 : PA3 :   TIM2_CH4
 * CH11 : PA6 :   TIM3_CH1
 * CH12 : PA7 :   TIM3_CH2
 */
#define GPIO_TIM1_CH1OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define GPIO_TIM4_CH1OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN12)
#define GPIO_TIM4_CH2OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH3OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define GPIO_TIM4_CH4OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN15)
#define GPIO_TIM2_CH3OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM2_CH4OUT    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM3_CH1OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN6)
#define GPIO_TIM3_CH2OUT    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN7)

#define DIRECT_PWM_OUTPUT_CHANNELS  10

#define GPIO_TIM1_CH1IN     GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN     GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN     GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN     GPIO_TIM1_CH4IN_2
#define GPIO_TIM4_CH1IN     GPIO_TIM4_CH1IN_2
#define GPIO_TIM4_CH2IN     GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH3IN     GPIO_TIM4_CH3IN_2
#define GPIO_TIM4_CH4IN     GPIO_TIM4_CH4IN_2
#define GPIO_TIM2_CH3IN     GPIO_TIM2_CH3IN_1
#define GPIO_TIM2_CH4IN     GPIO_TIM2_CH4IN_1
#define GPIO_TIM3_CH1IN     GPIO_TIM3_CH1IN_1
#define GPIO_TIM3_CH2IN     GPIO_TIM3_CH2IN_1

/* USB OTG FS
 *
 * PC4  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN4)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel */

/* PWM input driver. Use FMU backup pins attached to timer2 channel 1 */
#define PWMIN_TIMER		2
#define PWMIN_TIMER_CHANNEL	1
#define GPIO_TIM2_CH1IN     GPIO_TIM2_CH1IN_3
#define GPIO_PWM_IN		GPIO_TIM2_CH1IN

/* RC Chanle SBUS [UART4] */
#define RC_SERIAL_PORT      "/dev/ttyS3"
#define INVERT_RC_INPUT(_s)     while(0)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */

__END_DECLS
