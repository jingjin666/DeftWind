/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_defines.h
 *
 * Generally used magic defines
 */

#pragma once

#include <dp_log.h>
#include <math.h>

/* Get the name of the default value fiven the param name */
#define DP_PARAM_DEFAULT_VALUE_NAME(_name) PARAM_##_name##_DEFAULT

/* Shortcuts to define parameters when the default value is defined according to DP_PARAM_DEFAULT_VALUE_NAME */
#define DP_PARAM_DEFINE_INT32(_name) PARAM_DEFINE_INT32(_name, DP_PARAM_DEFAULT_VALUE_NAME(_name))
#define DP_PARAM_DEFINE_FLOAT(_name) PARAM_DEFINE_FLOAT(_name, DP_PARAM_DEFAULT_VALUE_NAME(_name))

#define DP_ERROR (-1)
#define DP_OK 0

#if defined(__DP_NUTTX) || defined(__DP_POSIX)
/*
 * Building for NuttX or POSIX
 */
#include <platforms/dp_includes.h>
/* Main entry point */
#define DP_MAIN_FUNCTION(_prefix) int _prefix##_task_main(int argc, char *argv[])
#if 0
/* Parameter handle datatype */
#include <systemlib/param/param.h>
typedef param_t dp_param_t;

/* Get value of parameter by name */
#define DP_PARAM_GET_BYNAME(_name, _destpt) param_get(param_find(_name), _destpt)
#endif
#else
#error "No target OS defined"
#endif

/*
 * NuttX Specific defines
 */
#if defined(__DP_NUTTX)

#define DP_ROOTFSDIR

/* XXX this is a hack to resolve conflicts with NuttX headers */
#if 0//!defined(__DP_TESTS)
#define isspace(c) \
	((c) == ' '  || (c) == '\t' || (c) == '\n' || \
	 (c) == '\r' || (c) == '\f' || c== '\v')
#endif

#define _DP_IOC(x,y) _IOC(x,y)

#define dp_statfs_buf_f_bavail_t int

#define DP_ISFINITE(x) isfinite(x)

// mode for open with O_CREAT
#define DP_O_MODE_777 0777
#define DP_O_MODE_666 0666
#define DP_O_MODE_600 0600

#ifndef PRIu64
#define PRIu64 "llu"
#endif
#ifndef PRId64
#define PRId64 "lld"
#endif

/*
 * POSIX Specific defines
 */
#elif defined(__DP_POSIX)

// Flag is meaningless on Linux
#define O_BINARY 0

// mode for open with O_CREAT
#define DP_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define DP_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define DP_O_MODE_600 (S_IRUSR | S_IWUSR)


// NuttX _IOC is equivalent to Linux _IO
#define _DP_IOC(x,y) _IO(x,y)

/* FIXME - Used to satisfy build */
//STM DocID018909 Rev 8 Sect 39.1 (Unique device ID Register)
#define UNIQUE_ID       0x1FFF7A10
#define STM32_SYSMEM_UID "SIMULATIONID"

/* FIXME - Used to satisfy build */
#define getreg32(a)    (*(volatile uint32_t *)(a))

__BEGIN_DECLS
extern long DP_TICKS_PER_SEC;
__END_DECLS

#define USEC_PER_TICK (1000000UL/DP_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#define dp_statfs_buf_f_bavail_t unsigned long

#define DP_ROOTFSDIR "rootfs"

#endif

/*
 * Defines for Linux
 */
#if defined(__DP_POSIX)
#define OK 0
#define ERROR -1

#define MAX_RAND 32767

#include <math.h>

/* Float defines of the standard double length constants  */
#define M_E_F			(float)M_E
#define M_LOG2E_F		(float)M_LOG2E
#define M_LOG10E_F		(float)M_LOG10E
#define M_LN2_F			(float)M_LN2
#define M_LN10_F		(float)M_LN10
#define M_PI_F			(float)M_PI
#define M_TWOPI_F       	(M_PI_F * 2.0f)
#define M_PI_2_F		(float)M_PI_2
#define M_PI_4_F		(float)M_PI_4
#define M_3PI_4_F		(float)2.3561944901923448370E0f
#define M_SQRTPI_F      	(float)1.77245385090551602792981f
#define M_1_PI_F		(float)M_1_PI
#define M_2_PI_F		(float)M_2_PI
#define M_2_SQRTPI_F		1.12837916709551257390f
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f
#define M_SQRT2_F		(float)M_SQRT2
#define M_SQRT1_2_F		(float)M_SQRT1_2
#define M_LN2LO_F       	1.9082149292705877000E-10f
#define M_LN2HI_F       	6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      	0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      	_M_LN2_F
#define M_INVLN2_F      	1.4426950408889633870E0f/* 1 / log(2)  */
#define M_DEG_TO_RAD 		0.01745329251994
#define M_RAD_TO_DEG 		57.2957795130823

#ifndef __DP_QURT

#if defined(__cplusplus)
#include <cmath>
#define DP_ISFINITE(x) std::isfinite(x)
#else
#define DP_ISFINITE(x) isfinite(x)
#endif
#endif

#endif

/*
 *Defines for all platforms
 */

/* wrapper for 2d matrices */
#define DP_ARRAY2D(_array, _ncols, _x, _y) (_array[_x * _ncols + _y])

/* wrapper for rotation matrices stored in arrays */
#define DP_R(_array, _x, _y) DP_ARRAY2D(_array, 3, _x, _y)
