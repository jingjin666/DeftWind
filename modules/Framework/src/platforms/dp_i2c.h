/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_i2c.h
 *
 * Includes device headers depending on the build target
 */

#pragma once

#define DP_I2C_M_READ           0x0001          /* read data, from slave to master */

#if defined (__DP_NUTTX)
/*
 * Building for NuttX
 */
#include <sys/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <chip.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include "up_internal.h"
#include "up_arch.h"

#define dp_i2c_msg_t i2c_msg_s

typedef struct i2c_master_s dp_i2c_dev_t;

#elif defined(__DP_POSIX)
#include <stdint.h>

#define I2C_M_READ           0x0001          /* read data, from slave to master */
#define I2C_M_TEN            0x0002          /* ten bit address */
#define I2C_M_NORESTART      0x0080          /* message should not begin with (re-)start of transfer */

// NOTE - This is a copy of the NuttX i2c_msg_s structure
typedef struct {
	uint16_t  addr;                  /* Slave address */
	uint16_t  flags;                 /* See I2C_M_* definitions */
	uint8_t  *buffer;
	int       length;
} dp_i2c_msg_t;

// NOTE - This is a copy of the NuttX i2c_ops_s structure
typedef struct {
	const struct dp_i2c_ops_t *ops; /* I2C vtable */
} dp_i2c_dev_t;

// FIXME - Empty defines for I2C ops
// Original version commented out
//#define I2C_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))
#define I2C_SETFREQUENCY(d,f)
//#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))
#define SPI_SELECT(d,id,s)

// FIXME - Stub implementation
// Original version commented out
//#define I2C_TRANSFER(d,m,c) ((d)->ops->transfer(d,m,c))
inline int I2C_TRANSFER(dp_i2c_dev_t *dev, dp_i2c_msg_t *msg, int count);
inline int I2C_TRANSFER(dp_i2c_dev_t *dev, dp_i2c_msg_t *msg, int count) { return 0; }
#else
#error "No target platform defined"
#endif
