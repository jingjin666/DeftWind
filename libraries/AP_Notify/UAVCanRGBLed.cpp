/*
  UAVCAN RGBLed driver
*/

/* LED driver for UAVCANRGBLed */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS

#include <AP_HAL_UAVRS/CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include "UAVCanRGBLed.h"

extern const AP_HAL::HAL& hal;

UAVCANRGBLed::UAVCANRGBLed():
    RGBLed(UAVCAN_LED_OFF, UAVCAN_LED_BRIGHT, UAVCAN_LED_MEDIUM, UAVCAN_LED_DIM)
{
    
}

bool UAVCANRGBLed::hw_init()
{
    bool ret = false;

    if (AP_BoardConfig_CAN::get_can_num_ifaces() >= 1) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                _uavcan = hal.can_mgr[i]->get_UAVCAN();
                printf("UAVCANRGBLed::hw_init ok can_idx[%d]\n", i);
                return true;
            }
        }
   }
   return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool UAVCANRGBLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb = {red, green, blue};
    _need_update = true;

    if(_uavcan != nullptr) {
        _uavcan->do_cyclic_rgbled(rgb.r, rgb.g, rgb.b);
    }
    
    return true;
}
void UAVCANRGBLed::_timer(void)
{
    if (!_need_update) {
        return;
    }

    _need_update = false;
}

#endif
