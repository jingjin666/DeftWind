#pragma once

#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// default pins and dividers
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && defined(CONFIG_ARCH_BOARD_UAVRS_V1)
 # define AP_BATT_VOLT_PIN				   15
 # define AP_BATT_CURR_PIN				   -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT	   9.64f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
 # define AP_BATT_COPTER_VOLT_PIN		    8
 # define AP_BATT_STEERING_GEAR_VOLT_PIN	9
#elif CONFIG_HAL_BOARD == HAL_BOARD_UAVRS && defined(CONFIG_ARCH_BOARD_UAVRS_V2)
 # define AP_BATT_VOLT_PIN				   10
 # define AP_BATT_CURR_PIN				   -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT	    17.9f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
 # define AP_BATT_COPTER_VOLT_PIN		    3
 # define AP_BATT_STEERING_GEAR_VOLT_PIN	9
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
 # define AP_BATT_COPTER_VOLT_PIN           -1
 # define AP_BATT_STEERING_GEAR_VOLT_PIN    -1
#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
 # define AP_BATT_COPTER_VOLT_PIN		    -1
 # define AP_BATT_STEERING_GEAR_VOLT_PIN	-1
#endif

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

    /// returns true if battery monitor provides current info
    bool has_current() const override;

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)	|| defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    AP_HAL::AnalogSource *_copter_volt_pin_analog_source;
	AP_HAL::AnalogSource *_steer_volt_pin_analog_source;
#endif
};
