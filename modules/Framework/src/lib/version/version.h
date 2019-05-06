/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file version.h
 *
 * Tools for system version detection.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef VERSION_H_
#define VERSION_H_

#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
#define	HW_ARCH "DPFMU_V1"
#endif

#ifdef CONFIG_ARCH_BOARD_UAVRS_V2
#define	HW_ARCH "DPFMU_V2"
#endif

#ifdef CONFIG_ARCH_BOARD_SITL
#define	HW_ARCH "LINUXTEST"
#endif

#endif /* VERSION_H_ */
