#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state) :
    AP_BattMonitor_Backend(mon, mon_state)
{
    _volt_pin_analog_source = hal.analogin->channel(mon._volt_pin[_state.instance]);
    _curr_pin_analog_source = hal.analogin->channel(mon._curr_pin[_state.instance]);
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)	|| defined(CONFIG_ARCH_BOARD_UAVRS_V2)
	_copter_volt_pin_analog_source = hal.analogin->channel(mon._copter_volt_pin[_state.instance]);
	_steer_volt_pin_analog_source = hal.analogin->channel(mon._steer_volt_pin[_state.instance]);
#endif
    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // get voltage
    _state.voltage = _volt_pin_analog_source->voltage_average() * _mon._volt_multiplier[_state.instance];
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1)	|| defined(CONFIG_ARCH_BOARD_UAVRS_V2)
	_state.copter_voltage = _copter_volt_pin_analog_source->voltage_average() * _mon._volt_multiplier[_state.instance];
	_state.steer_voltage = _steer_volt_pin_analog_source->voltage_average() * _mon._volt_multiplier[_state.instance];
#endif

    // read current
    if (has_current()) {
        // calculate time since last current read
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _curr_pin_analog_source->set_pin(_mon._curr_pin[_state.instance]);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average()-_mon._curr_amp_offset[_state.instance])*_mon._curr_amp_per_volt[_state.instance];

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            _state.current_total_mah += _state.current_amps * dt * 0.0000002778f;
        }

        // record time
        _state.last_time_micros = tnow;
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
    return (_mon.get_type(_state.instance) == AP_BattMonitor::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
}
