/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_device.h
 *
 * Generic device / sensor interface.
 */

#ifndef _DRV_DEVICE_H
#define _DRV_DEVICE_H

#include <stdint.h>
#include <sys/ioctl.h>
#if 0
#include "drv_sensor.h"
#include "drv_orb_dev.h"
#endif
/*
 * ioctl() definitions
 */

#define _DEVICEIOCBASE		(0x100)
#define _DEVICEIOC(_n)		(_DP_IOC(_DEVICEIOCBASE, _n))

/** ask device to stop publishing */
#define DEVIOCSPUBBLOCK	_DEVICEIOC(0)

/** check publication block status */
#define DEVIOCGPUBBLOCK	_DEVICEIOC(1)

/**
 * Return device ID, to enable matching of configuration parameters
 * (such as compass offsets) to specific sensors
 */
#define DEVIOCGDEVICEID	_DEVICEIOC(2)

#ifdef __DP_POSIX

#ifndef SIOCDEVPRIVATE
#define SIOCDEVPRIVATE 1
#endif

#define DIOC_GETPRIV    SIOCDEVPRIVATE
#endif

#endif /* _DRV_DEVICE_H */
