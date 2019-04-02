/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_time.h
 *
 * Includes device headers depending on the build target
 */

#pragma once

#include <sys/types.h>
#include <time.h>

#if defined(__DP_LINUX) || defined(__DP_NUTTX)

#define dp_clock_gettime clock_gettime
#define dp_clock_settime clock_settime

#endif
