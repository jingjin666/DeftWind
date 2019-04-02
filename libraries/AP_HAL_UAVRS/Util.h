#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_UAVRS_Namespace.h"

class UAVRS::Util : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};
