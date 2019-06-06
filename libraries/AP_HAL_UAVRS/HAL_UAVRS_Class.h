#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_UAVRS_Namespace.h"
#include "PrivateMember.h"

class HAL_UAVRS : public AP_HAL::HAL {
public:
    HAL_UAVRS();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
private:
    UAVRS::PrivateMember *_member;
};

void hal_uavrs_set_priority(uint8_t priority);

