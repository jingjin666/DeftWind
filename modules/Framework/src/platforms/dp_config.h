/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_config.h
   Configuration flags used in code.
 */

#pragma once

#if defined(__DP_NUTTX)

#include <nuttx/config.h>
#include <board_config.h>

#elif defined (__DP_POSIX)

#define CONFIG_NFILE_STREAMS 1
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_LPWORK 1
#define CONFIG_ARCH_BOARD_SITL 1

/** time in ms between checks for work in work queues **/
#define CONFIG_SCHED_WORKPERIOD 50000

#define CONFIG_SCHED_INSTRUMENTATION 1
#define CONFIG_MAX_TASKS 32

#endif
