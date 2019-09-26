#pragma once

#include "ap_version.h"

#define THISFIRMWARE "UAVRS PLANE V2.1.1"
#define FIRMWARE_VERSION 2,1,1,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
