#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>

class AP_Arming {
public:
    enum ArmingChecks {
        ARMING_CHECK_NONE       = 0x0000,
        ARMING_CHECK_ALL        = 0x0001,
        ARMING_CHECK_BARO       = 0x0002,
        ARMING_CHECK_COMPASS    = 0x0004,
        ARMING_CHECK_GPS        = 0x0008,
        ARMING_CHECK_INS        = 0x0010,
        ARMING_CHECK_PARAMETERS = 0x0020,
        ARMING_CHECK_RC         = 0x0040,
        ARMING_CHECK_VOLTAGE    = 0x0080,
        ARMING_CHECK_BATTERY    = 0x0100,
        ARMING_CHECK_AIRSPEED   = 0x0200,
        ARMING_CHECK_LOGGING    = 0x0400,
        ARMING_CHECK_SWITCH     = 0x0800,
        ARMING_CHECK_GPS_CONFIG = 0x1000,
    };

    enum ArmingMethod {
        NONE = 0,
        RUDDER,
        MAVLINK
    };

    enum ArmingRequired {
        NO           = 0,
        YES_MIN_PWM  = 1,
        YES_ZERO_PWM = 2
    };

    AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
              const AP_BattMonitor &battery);

    // these functions should not be used by Copter which holds the armed state in the motors library
    ArmingRequired arming_required();
    virtual bool arm(uint8_t method);
    bool disarm();
    bool is_armed();

    // get bitmask of enabled checks
    uint16_t get_enabled_checks();

    // pre_arm_checks() is virtual so it can be modified in a vehicle specific subclass
    virtual bool pre_arm_checks(bool report);

    // some arming checks have side-effects, or require some form of state
    // change to have occurred, and thus should not be done as pre-arm
    // checks.  Those go here:
    bool arm_checks(uint8_t method);

    // get expected magnetic field strength
    uint16_t compass_magfield_expected() const;

    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // Parameters
    AP_Int8                 require;
    AP_Int16                checks_to_perform;      // bitmask for which checks are required
    AP_Float                accel_error_threshold;
    AP_Float                _min_voltage[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_Float                _steer_voltage[2]; // max and min
    AP_Float                _copter_voltage;

    // references
    const AP_AHRS           &ahrs;
    const AP_Baro           &barometer;
    Compass                 &_compass;
    const AP_BattMonitor    &_battery;

    // internal members
    bool                    armed:1;
    bool                    logging_available:1;
    uint8_t                 arming_method;          // how the vehicle was armed
    uint32_t                last_accel_pass_ms[INS_MAX_INSTANCES];
    uint32_t                last_gyro_pass_ms[INS_MAX_INSTANCES];

public:
    bool barometer_checks(bool report);

    bool airspeed_checks(bool report);

    bool logging_checks(bool report);

    virtual bool ins_checks(bool report);

    virtual bool compass_checks(bool report);

    virtual bool gps_checks(bool report);

    bool battery_checks(bool report);

    bool hardware_safety_check(bool report);

    bool board_voltage_checks(bool report);

    bool manual_transmitter_checks(bool report);

    virtual enum HomeState home_status() const = 0;

    bool ppk_checks(bool report);
};
