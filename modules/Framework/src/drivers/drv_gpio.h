/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_gpio.h
 *
 * Generic GPIO ioctl interface.
 */

#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <sys/ioctl.h>

#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
/*
 * UAVRSv1 GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
# define GPIO_SERVO_1			(1<<0)		/**< servo 1 output */
# define GPIO_SERVO_2			(1<<1)		/**< servo 2 output */
# define GPIO_SERVO_3			(1<<2)		/**< servo 3 output */
# define GPIO_SERVO_4			(1<<3)		/**< servo 4 output */
# define GPIO_SERVO_5			(1<<4)		/**< servo 5 output */
# define GPIO_SERVO_6			(1<<5)		/**< servo 6 output */
# define GPIO_SERVO_7			(1<<6)		/**< servo 7 output */
# define GPIO_SERVO_8			(1<<7)		/**< servo 8 output */

# define GPIO_CAMERA_TRIGGER	(1<<8)		/**< relay CAMERA TRIGGER */
# define GPIO_CAMERA_FEEDBACK	(1<<9)		/**< input CAMERA FEEDBACK */


/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define FMU_DEVICE_PATH	"/dev/fmu"

#endif

#ifdef CONFIG_ARCH_BOARD_UAVRS_V2
/*
 * UAVRSv2 GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
# define GPIO_SERVO_1			(1<<0)		/**< servo 1 output */
# define GPIO_SERVO_2			(1<<1)		/**< servo 2 output */
# define GPIO_SERVO_3			(1<<2)		/**< servo 3 output */
# define GPIO_SERVO_4			(1<<3)		/**< servo 4 output */
# define GPIO_SERVO_5			(1<<4)		/**< servo 5 output */
# define GPIO_SERVO_6			(1<<5)		/**< servo 6 output */
# define GPIO_SERVO_7			(1<<6)		/**< servo 7 output */
# define GPIO_SERVO_8			(1<<7)		/**< servo 8 output */

# define GPIO_CAMERA_TRIGGER	(1<<8)		/**< relay CAMERA TRIGGER */
# define GPIO_CAMERA_FEEDBACK	(1<<9)		/**< input CAMERA FEEDBACK */


/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define FMU_DEVICE_PATH	"/dev/fmu"

#endif


#ifdef CONFIG_ARCH_BOARD_SITL
/* no GPIO driver on the SITL configuration */
#endif

#if !defined(CONFIG_ARCH_BOARD_UAVRS_V1) && \
	!defined(CONFIG_ARCH_BOARD_UAVRS_V2)
# error No CONFIG_ARCH_BOARD_xxxx set
#endif
/*
 * IOCTL definitions.
 *
 * For all ioctls, the (arg) argument is a bitmask of GPIOs to be affected
 * by the operation, with the LSB being the lowest-numbered GPIO.
 *
 * Note that there may be board-specific relationships between GPIOs;
 * applications using GPIOs should be aware of this.
 */
#define _GPIOCBASE	0x2700
#define GPIOC(_x)	_IOC(_GPIOCBASE, _x)

/** reset all board GPIOs to their default state */
#define GPIO_RESET	GPIOC(0)

/** configure the board GPIOs in (arg) as outputs */
#define GPIO_SET_OUTPUT	GPIOC(1)

/** configure the board GPIOs in (arg) as inputs */
#define GPIO_SET_INPUT	GPIOC(2)

/** configure the board GPIOs in (arg) for the first alternate function (if supported) */
#define GPIO_SET_ALT_1	GPIOC(3)

/** configure the board GPIO (arg) for the second alternate function (if supported) */
#define GPIO_SET_ALT_2	GPIOC(4)

/** configure the board GPIO (arg) for the third alternate function (if supported) */
#define GPIO_SET_ALT_3	GPIOC(5)

/** configure the board GPIO (arg) for the fourth alternate function (if supported) */
#define GPIO_SET_ALT_4	GPIOC(6)

/** set the GPIOs in (arg) */
#define GPIO_SET	GPIOC(10)

/** clear the GPIOs in (arg) */
#define GPIO_CLEAR	GPIOC(11)

/** read all the GPIOs and return their values in *(uint32_t *)arg */
#define GPIO_GET	GPIOC(12)

#define GPIO_SENSOR_RAIL_RESET	GPIOC(13)

#define GPIO_PERIPHERAL_RAIL_RESET	GPIOC(14)

/** configure the board GPIOs in (arg) as outputs, initially low */
#define GPIO_SET_OUTPUT_LOW	GPIOC(15)

/** configure the board GPIOs in (arg) as outputs, initially high */
#define GPIO_SET_OUTPUT_HIGH	GPIOC(16)

/** set the duty cycle as an integer percentage on the IMU heater pin if available */
#define GPIO_SET_HEATER_DUTY_CYCLE	GPIOC(17)

#endif /* _DRV_GPIO_H */
