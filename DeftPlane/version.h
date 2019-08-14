#pragma once

#include "ap_version.h"

#define THISFIRMWARE "Uav-rs plane v2.0"
#define FIRMWARE_VERSION 2,0,0,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
