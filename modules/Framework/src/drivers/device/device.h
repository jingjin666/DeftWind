/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#ifdef __DP_NUTTX
#include "nuttx/device_nuttx.h"
#elif defined (__DP_POSIX)
#include "posix/vdev.h"
#endif

