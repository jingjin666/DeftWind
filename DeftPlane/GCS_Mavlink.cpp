#include "GCS_Mavlink.h"

#include "Plane.h"
#include "version.h"

void Plane::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status;
    uint32_t custom_mode = control_mode;
    
    if (failsafe.state != FAILSAFE_NONE || failsafe.low_battery || failsafe.adsb) {
        system_status = MAV_STATE_CRITICAL;
    } else if (plane.crash_state.is_crashed) {
        system_status = MAV_STATE_EMERGENCY;
    } else if (is_flying()) {
        system_status = MAV_STATE_ACTIVE;
    } else {
        system_status = MAV_STATE_STANDBY;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
    case TRAINING:
    case ACRO:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case CRUISE:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case NO_GPS_RTL:
    case QRTL:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (!training_manual_pitch || !training_manual_roll) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing != STICK_MIXING_DISABLED && control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    // we are armed if we are not initialising
    if (control_mode != INITIALISING && arming.is_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    gcs().chan(chan-MAVLINK_COMM_0).send_heartbeat(quadplane.get_mav_type(),
                                                   base_mode,
                                                   custom_mode,
                                                   system_status);
}

void Plane::send_indicator(mavlink_channel_t chan)
{
    float rtk_heading;
    float compass_heading;
    struct indicator_status indicator;
    bool use_gps_head = gps.have_heading() && gps.heading_status() == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;

    indicator.rtk_head_status = false;
    rtk_heading = -1.0f;

    for(uint8_t i=0; i<2; i++){
        if(gps.get_gps_type(i) == AP_GPS_Backend::DEVTYPE_OEM719 && 
            gps.heading_status(i) == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED){
            indicator.rtk_head_status = true;
            rtk_heading = gps.get_heading(i);
        }
    }

    if(compass.use_for_yaw()){
        compass_heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned())*RAD_TO_DEG;
        if(compass_heading < 0 ){
            compass_heading = 360.0f + compass_heading;
        }
    }else{
        compass_heading = -1.0f;
    }

    indicator.barometer_status = arming.barometer_checks(false);
    indicator.ins_status = arming.ins_checks(false);
    indicator.compass_status = arming.compass_checks(false);
    indicator.gps_status = arming.gps_checks(false);
    indicator.plane_battery_status = arming.battery_checks(false);
    indicator.sdcard_status = arming.logging_checks(true);
    indicator.radio_status = arming.manual_transmitter_checks(false);
    indicator.board_voltage_status = true;//arming.board_voltage_checks(false);
    indicator.hardware_safety_status = true;//arming.hardware_safety_check(false);
    indicator.eeprom_status = (load_param_flag == LOAD_PARAM_OK ? true:false);
    indicator.airspeed_status = arming.airspeed_checks(false);
    indicator.ppk_status = arming.ppk_checks(false);

    float heading_error = fabs(rtk_heading - compass_heading);
    heading_error = heading_error > 180 ? 360 - heading_error : heading_error;

     if(!indicator.compass_status || !use_gps_head || (plane.compass.use_for_yaw() && use_gps_head && heading_error > 90.0f)){
        indicator.rtk_compass_diff = false;
    }else{
        indicator.rtk_compass_diff = true;
    }

    gcs().chan(chan-MAVLINK_COMM_0).send_indicator(*((uint32_t*)&indicator), rtk_heading, compass_heading, g.fly_odometer_km, g.fly_time_hour);
}

void Plane::send_p900_id(mavlink_channel_t chan)
{
    gcs().chan(chan-MAVLINK_COMM_0).send_p900_id(return_p900_id, strlen((char*)return_p900_id));
}

void Plane::send_event_report(mavlink_channel_t chan){
    gcs().chan(chan-MAVLINK_COMM_0).send_event_report(event_report);
}

void Plane::send_attitude(mavlink_channel_t chan)
{
    float r = ahrs.roll;
    float p = ahrs.pitch - radians(g.pitch_trim_cd*0.01f);
    float y = ahrs.yaw;
    
    if (quadplane.tailsitter_active()) {
        r = quadplane.ahrs_view->roll;
        p = quadplane.ahrs_view->pitch;
        y = quadplane.ahrs_view->yaw;
    }
    
    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        r,
        p,
        y,
        omega.x,
        omega.y,
        omega.z);
}

void Plane::send_aoa_ssa(mavlink_channel_t chan)
{
    mavlink_msg_aoa_ssa_send(
        chan,
        micros(),
        ahrs.getAOA(),
        ahrs.getSSA());
}

#if GEOFENCE_ENABLED == ENABLED
void Plane::send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


void Plane::send_extended_status1(mavlink_channel_t chan)
{
    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    update_sensor_status_flags();
    
    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(20000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        battery.copter_voltage() * 1000, //mv
        battery.steer_voltage() * 1000, //mv
        0,
        0);
}

void Plane::send_location(mavlink_channel_t chan)
{
    uint32_t fix_time_ms;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time_ms = gps.last_fix_time_ms();
    } else {
        fix_time_ms = millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time_ms,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        current_loc.alt * 10UL,         // millimeters above sea level
        relative_altitude * 1.0e3f,    // millimeters above ground
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.z * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

void Plane::send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01f,
        nav_pitch_cd * 0.01f,
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        MIN(auto_state.wp_distance, UINT16_MAX),
        altitude_error_cm * 0.01f,
        airspeed_error * 100,
        nav_controller->crosstrack_error());
}

void Plane::send_position_target_global_int(mavlink_channel_t chan)
{
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL_INT, // targets are always global altitude
        0xFFF8, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        next_WP_loc.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}


void Plane::send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f),
        0,
        0,
        0,
        0,
        receiver_rssi);
}

void Plane::send_vfr_hud(mavlink_channel_t chan)
{
    float aspeed;
    if (airspeed.enabled()) {
        aspeed = airspeed.get_airspeed();
    } else if (!ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0;
    }
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        ahrs.groundspeed(),
        (ahrs.yaw_sensor / 100) % 360,
        abs(throttle_percentage()),
        current_loc.alt / 100.0f,
        (g2.soaring_controller.is_active() ? g2.soaring_controller.get_vario_reading() : barometer.get_climb_rate()));
}

/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */

// report simulator state
void Plane::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

void Plane::send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        0);
}

void Plane::send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

/*
  send RPM packet
 */
void NOINLINE Plane::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}

/*
  send PID tuning message
 */
void Plane::send_pid_tuning(mavlink_channel_t chan)
{
    const Vector3f &gyro = ahrs.get_gyro();
    const DataFlash_Class::PID_Info *pid_info;
    if (g.gcs_pid_mask & 1) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        } else {
            pid_info = &rollController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL, 
                                    pid_info->desired,
                                    degrees(gyro.x),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        } else {
            pid_info = &pitchController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH, 
                                    pid_info->desired,
                                    degrees(gyro.y),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        if (quadplane.in_vtol_mode()) {
            pid_info = &quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        } else {
            pid_info = &yawController.get_pid_info();
        }
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info->desired,
                                    degrees(gyro.z),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        pid_info = &steerController.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEER, 
                                    pid_info->desired,
                                    degrees(gyro.z),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if ((g.gcs_pid_mask & 0x10) && (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND)) {
        pid_info = landing.get_pid_info();
        if (pid_info != nullptr) {
            mavlink_msg_pid_tuning_send(chan, PID_TUNING_LANDING,
                                        pid_info->desired,
                                        gyro.z,
                                        pid_info->FF,
                                        pid_info->P,
                                        pid_info->I,
                                        pid_info->D);
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

void Plane::send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
}

uint8_t GCS_MAVLINK_Plane::sysid_my_gcs() const
{
    return plane.g.sysid_my_gcs;
}

uint32_t GCS_MAVLINK_Plane::telem_delay() const
{
    return (uint32_t)(plane.g.telem_delay);
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    if (telemetry_delayed()) {
        return false;
    }

    // if we don't have at least 0.2ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!plane.in_mavlink_delay && plane.scheduler.time_available_usec() < 200) {
        plane.gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        plane.send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        plane.send_extended_status1(chan);
        CHECK_PAYLOAD_SIZE2(POWER_STATUS);
        send_power_status();
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        plane.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        plane.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(plane.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            plane.send_nav_controller_output(chan);
        }
        break;

    case MSG_POSITION_TARGET_GLOBAL_INT:
        if (plane.control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(POSITION_TARGET_GLOBAL_INT);
            plane.send_position_target_global_int(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(plane.gps);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(plane.gps);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS);
        send_radio_in(plane.receiver_rssi);
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw(false);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        plane.send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu(plane.ins, plane.compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure(plane.barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets(plane.ins, plane.compass, plane.barometer);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        plane.send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        queued_waypoint_send();
        break;

    case MSG_FENCE_STATUS:
#if GEOFENCE_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        plane.send_fence_status(chan);
#endif
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(plane.ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        plane.send_simstate(chan);
        CHECK_PAYLOAD_SIZE2(AHRS2);
        send_ahrs2(plane.ahrs);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        plane.send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder_downward(plane.rangefinder);
        CHECK_PAYLOAD_SIZE(DISTANCE_SENSOR);
        send_distance_sensor_downward(plane.rangefinder);
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
#endif
        break;

    case MSG_CAMERA_FEEDBACK:
#if CAMERA == ENABLED
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        plane.camera.send_feedback(chan);
#endif
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2(plane.battery);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        plane.send_wind(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        plane.camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_EKF_STATUS_REPORT:
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        plane.ahrs.send_ekf_status_report(chan);
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        plane.camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        plane.send_pid_tuning(chan);
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration(plane.ins);
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        plane.send_rpm(chan);
        break;

    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
        break;

    case MSG_MAG_CAL_PROGRESS:
        plane.compass.send_mag_cal_progress(chan);
        break;

    case MSG_MAG_CAL_REPORT:
        plane.compass.send_mag_cal_report(chan);
        break;

    case MSG_ADSB_VEHICLE:
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
        break;
    case MSG_BATTERY_STATUS:
        send_battery_status(plane.battery);
        break;
    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        plane.send_aoa_ssa(chan);
        break;
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;
    case MSG_INDICATOR:
        CHECK_PAYLOAD_SIZE(INDICATOR_STATUS);
        plane.send_indicator(chan);
        break;
    case MSG_P900_ID:
        plane.send_p900_id(chan);
        break;
    case MSG_EVENT_REPORT:
        plane.send_event_report(chan);
        break;
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK, streamRates[9],  5),
    AP_GROUPEND
};

void
GCS_MAVLINK_Plane::data_stream_send(void)
{
    plane.gcs_out_of_time = false;

    if (!plane.in_mavlink_delay) {
        plane.DataFlash.handle_log_send(*this);
		plane.DataFlash.handle_raw_data_send(*this);
		plane.DataFlash.handle_pos_data_send(*this);
    }

    send_queued_parameters();

    if (plane.gcs_out_of_time) return;

    if (plane.in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
        send_message(MSG_POSITION_TARGET_GLOBAL_INT);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_SERVO_OUTPUT_RAW);
        send_message(MSG_RADIO_IN);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        send_message(MSG_RPM);
        send_message(MSG_AOA_SSA);

        if (plane.control_mode != MANUAL) {
            send_message(MSG_PID_TUNING);
        }
        send_message(MSG_LANDING);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_WIND);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
#if AP_TERRAIN_AVAILABLE
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_MAG_CAL_REPORT);
        send_message(MSG_MAG_CAL_PROGRESS);
        send_message(MSG_BATTERY2);
        send_message(MSG_BATTERY_STATUS);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_EKF_STATUS_REPORT);
        send_message(MSG_GIMBAL_REPORT);
        send_message(MSG_VIBRATION);
    }

    if (plane.gcs_out_of_time) return;

    if (stream_trigger(STREAM_ADSB)) {
        send_message(MSG_ADSB_VEHICLE);
    }
}


/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (plane.control_mode != GUIDED) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (plane.guided_WP_loc.flags.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.flags.relative_alt = 0;
    }

    plane.set_guided_WP();
    return true;
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.flags.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.flags.relative_alt = false;
    plane.next_WP_loc.flags.terrain_alt = cmd.content.location.flags.terrain_alt;
    plane.reset_offset_altitude();
}

void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                        mavlink_message_t &msg)
{
    plane.avoidance_adsb.handle_msg(msg);
    GCS_MAVLINK::packetReceived(status, msg);
}

void GCS_MAVLINK_Plane::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_INT:
    {
        // decode
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        switch(packet.command) {

        case MAV_CMD_DO_REPOSITION:
            // sanity check location
            if (!check_latlng(packet.x, packet.y)) {
                result = MAV_RESULT_FAILED;
                break;
            }

            Location requested_position {};
            requested_position.lat = packet.x;
            requested_position.lng = packet.y;

            // check the floating representation for overflow of altitude
            if (fabsf(packet.z * 100.0f) >= 0x7fffff) {
                result = MAV_RESULT_FAILED;
                break;
            }
            requested_position.alt = (int32_t)(packet.z * 100.0f);

            // load option flags
            if (packet.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                requested_position.flags.relative_alt = 1;
            }
            else if (packet.frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
                requested_position.flags.terrain_alt = 1;
            }
            else if (packet.frame != MAV_FRAME_GLOBAL_INT) {
                // not a supported frame
                break;
            }

            if (is_zero(packet.param4)) {
                requested_position.flags.loiter_ccw = 0;
            } else {
                requested_position.flags.loiter_ccw = 1;
            }

            if (location_sanitize(plane.current_loc, requested_position)) {
                // if the location wasn't already sane don't load it
                result = MAV_RESULT_FAILED; // failed as the location is not valid
                break;
            }

            // location is valid load and set
            if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
                (plane.control_mode == GUIDED)) {
                plane.set_mode(GUIDED, MODE_REASON_GCS_COMMAND);
                plane.guided_WP_loc = requested_position;

                // add home alt if needed
                if (plane.guided_WP_loc.flags.relative_alt) {
                    plane.guided_WP_loc.alt += plane.home.alt;
                    plane.guided_WP_loc.flags.relative_alt = 0;
                }

                plane.set_guided_WP();

                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED; // failed as we are not in guided
            }
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command

        switch(packet.command) {

        case MAV_CMD_DO_CHANGE_SPEED:
            // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
            // controlled modes (e.g., MANUAL, TRAINING)
            // this command should be ignored since it comes in from GCS
            // or a companion computer:
            result = MAV_RESULT_FAILED;
            if (plane.control_mode != GUIDED && plane.control_mode != AUTO && plane.control_mode != AVOID_ADSB) {
                // failed
                break;
            }

            AP_Mission::Mission_Command cmd;
            if (AP_Mission::mavlink_cmd_long_to_mission_cmd(packet, cmd)
                    == MAV_MISSION_ACCEPTED) {
                plane.do_change_speed(cmd);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            plane.set_mode(LOITER, MODE_REASON_GCS_COMMAND);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:{
        #if 0
            plane.set_mode(RTL, MODE_REASON_GCS_COMMAND);
            result = MAV_RESULT_ACCEPTED;
        #endif
            //Fly to the emergency point.
            // set current command
            if(plane.control_mode == GUIDED && plane.mission.starts_check_misson_cmd() && plane.arming.is_armed()){
                plane.set_mode(AUTO, MODE_REASON_GCS_COMMAND);
            }

            if(plane.control_mode == AUTO && plane.arming.is_armed() && plane.mission.starts_check_misson_cmd()){
                uint16_t seq = plane.mission.num_commands() - 9;
                if (plane.mission.set_current_cmd(seq)) {
                    plane.camera.set_trigger_distance(0);
                    mavlink_msg_mission_current_send(chan, seq);
                    result = MAV_RESULT_ACCEPTED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_RETURN);
                }else{
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_RETURN_REFUSE);
                }
            }else{
                result = MAV_RESULT_FAILED;
                mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_RETURN_REFUSE);
            }
            break;
        }

        case MAV_CMD_NAV_LAND:{
            //Fly to the emergency next point.
            // set current command
            uint16_t seq = plane.mission.num_commands() - 6;
            float alt = 0.0f;
            plane.ahrs.get_relative_position_D_home(alt);

            if(plane.control_mode == AUTO
                && plane.mission.get_current_nav_cmd().type == MAV_POINT_ZERO_EMERGENCY_WAYPOINT
                && plane.loiter.sum_cd > 1 
                && plane.mission.starts_check_misson_cmd()){
                if (plane.mission.set_current_cmd(seq)) {
                    mavlink_msg_mission_current_send(chan, seq);
                    result = MAV_RESULT_ACCEPTED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_LAND);
                }else{
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_LAND_REFUSE);
                }
            }else if (plane.control_mode == AUTO
                       && plane.mission.get_current_nav_cmd().type == MAV_POINT_VTOL_TAKEOFF
                       && -alt < plane.mission.get_current_nav_cmd().content.location.alt/100){
                       plane.set_mode(QLAND, MODE_REASON_GCS_COMMAND);
                       result = MAV_RESULT_ACCEPTED;
                       mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_LAND);
            }else{
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_LAND_REFUSE);
            }
            break;
        }

        case MAV_CMD_NAV_TAKEOFF: {
        #if 0
            // user takeoff only works with quadplane code for now
            // param7 : altitude [metres]
            float takeoff_alt = packet.param7;
            if (plane.quadplane.available() && plane.quadplane.do_user_takeoff(takeoff_alt)) {
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
        #endif
            if(plane.control_mode == QSTABILIZE && plane.arming.is_armed()){
                if(plane.mission.starts_check_misson_cmd()){
                    plane.mission.reset();
                    //Our first point is takeoff, so we can switch into the auto mode.
                    plane.set_mode(AUTO, MODE_REASON_GCS_COMMAND);
                    result = MAV_RESULT_ACCEPTED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_TAKEOFF);
                }else{
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_TAKEOFF_REFUSE);
                }

            }else{
                result = MAV_RESULT_FAILED;
                mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_TAKEOFF_REFUSE);
            }
            break;
        }
            
#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
        case MAV_CMD_DO_SET_ROI:
            // sanity check location
            if (!check_latlng(packet.param5, packet.param6)) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (plane.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    plane.camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                plane.camera_mount.set_roi_target(roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            plane.camera.configure(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6,
                                   packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            plane.camera.control(packet.param1,
                                 packet.param2,
                                 packet.param3,
                                 packet.param4,
                                 packet.param5,
                                 packet.param6);
            result = MAV_RESULT_ACCEPTED;
            break;

      case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            plane.camera.set_trigger_distance(packet.param1);
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED

        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            plane.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
            result = MAV_RESULT_ACCEPTED;
#endif
            break;

        case MAV_CMD_MISSION_START:
            plane.set_mode(AUTO, MODE_REASON_GCS_COMMAND);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            plane.in_calibration = true;
            if (is_equal(packet.param1,1.0f)) {
                /*
                  gyro calibration
                 */
                if (hal.util->get_soft_armed()) {
                    send_text(MAV_SEVERITY_WARNING, "No calibration while armed");
                    result = MAV_RESULT_FAILED;
                    break;
                }
                plane.ins.init_gyro();
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                /*
                  baro and airspeed calibration
                 */
                if (hal.util->get_soft_armed() && plane.is_flying()) {
                    send_text(MAV_SEVERITY_WARNING, "No calibration while flying");
                    result = MAV_RESULT_FAILED;
                    break;
                }
                plane.init_barometer(false);
                if (plane.airspeed.enabled()) {
                    plane.zero_airspeed(false);
                }
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param4,1.0f)) {
                /*
                  radio trim
                 */
                if (hal.util->get_soft_armed()) {
                    send_text(MAV_SEVERITY_WARNING, "No calibration while armed");
                    result = MAV_RESULT_FAILED;
                    break;
                }
                plane.trim_radio();
                result = MAV_RESULT_ACCEPTED;
            } else if (is_equal(packet.param5,1.0f)) {
                /*
                  accel calibration
                 */
                if (hal.util->get_soft_armed()) {
                    send_text(MAV_SEVERITY_WARNING, "No calibration while armed");
                    result = MAV_RESULT_FAILED;
                    break;
                }
                result = MAV_RESULT_ACCEPTED;
                // start with gyro calibration
                plane.ins.init_gyro();
                // reset ahrs gyro bias
                if (plane.ins.gyro_calibrated_ok_all()) {
                    plane.ahrs.reset_gyro_drift();
                } else {
                    result = MAV_RESULT_FAILED;
                }
                plane.ins.acal_init();
                plane.ins.get_acal()->start(this);

            } else if (is_equal(packet.param5,2.0f)) {
                /*
                  ahrs trim
                 */
                if (hal.util->get_soft_armed()) {
                    send_text(MAV_SEVERITY_WARNING, "No calibration while armed");
                    result = MAV_RESULT_FAILED;
                    break;
                }
                // start with gyro calibration
                plane.ins.init_gyro();
                // accel trim
                float trim_roll, trim_pitch;
                if(plane.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    plane.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            }
            else {
                    send_text(MAV_SEVERITY_WARNING, "Unsupported preflight calibration");
            }
            plane.in_calibration = false;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                if(!plane.arming.is_armed()){
                    // run pre_arm_checks and arm_checks and display failures
                    plane.set_mode(QSTABILIZE, MODE_REASON_GCS_COMMAND);
                    plane.mission.reset();
                    if(plane.control_mode == QSTABILIZE){
                        if (plane.arm_motors(AP_Arming::MAVLINK)) {
                            result = MAV_RESULT_ACCEPTED;
                            mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_ARM);
                        } else {
                            result = MAV_RESULT_FAILED;
                            mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_ARM_REFUSE);
                        }
                    }else{
                        result = MAV_RESULT_FAILED;
                        mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_ARM_REFUSE);
                    }
                }else{
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_ARM_REFUSE);
                }
            } else if (is_zero(packet.param1))  {
                if (plane.disarm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_DISARM);
                } else {
                    result = MAV_RESULT_FAILED;
                    mavlink_msg_plane_event_report_send(chan, PLANE_EVENT_REPORT_DISARM_REFUSE);
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (plane.home_is_set != HOME_UNSET) {
                send_home(plane.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                plane.set_mode(MANUAL, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                plane.set_mode(AUTO, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                plane.set_mode(FLY_BY_WIRE_A, MODE_REASON_GCS_COMMAND);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            result = handle_preflight_reboot(packet, plane.quadplane.enable != 0);
            break;

        case MAV_CMD_DO_LAND_START:
            result = MAV_RESULT_FAILED;
            
            // attempt to switch to next DO_LAND_START command in the mission
            if (plane.mission.jump_to_landing_sequence()) {
                plane.set_mode(AUTO, MODE_REASON_UNKNOWN);
                result = MAV_RESULT_ACCEPTED;
            } 
            break;

        case MAV_CMD_DO_GO_AROUND:
            result = MAV_RESULT_FAILED;

            if (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
                // Initiate an aborted landing. This will trigger a pitch-up and
                // climb-out to a safe altitude holding heading then one of the
                // following actions will occur, check for in this order:
                // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
                //      increment mission index to execute it
                // - else if DO_LAND_START is available, jump to it
                // - else decrement the mission index to repeat the landing approach

                if (!is_zero(packet.param1)) {
                    plane.auto_state.takeoff_altitude_rel_cm = packet.param1 * 100;
                }
                if (plane.landing.request_go_around()) {
                    plane.auto_state.next_wp_no_crosstrack = true;
                    result = MAV_RESULT_ACCEPTED;
                }
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
            result = MAV_RESULT_ACCEPTED;
            
            if (!plane.geofence_present()) {
                gcs().send_text(MAV_SEVERITY_NOTICE,"Fence not configured");
                result = MAV_RESULT_FAILED;
            } else {
                switch((uint16_t)packet.param1) {
                case 0:
                    if (! plane.geofence_set_enabled(false, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED;
                    }
                    break;
                case 1:
                    if (! plane.geofence_set_enabled(true, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED; 
                    }
                    break;
                case 2: //disable fence floor only 
                    if (! plane.geofence_set_floor_enabled(false)) {
                        result = MAV_RESULT_FAILED;
                    } else {
                        gcs().send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
                }
            }
            break;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f)) {
                plane.init_home();
            } else {
                if (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7)) {
                    // don't allow the 0,0 position
                    break;
                }
                // sanity check location
                if (!check_latlng(packet.param5,packet.param6)) {
                    break;
                }
                Location new_home_loc {};
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                plane.ahrs.set_home(new_home_loc);
                plane.home_is_set = HOME_SET_NOT_LOCKED;
                plane.Log_Write_Home_And_Origin();
                gcs().send_home(new_home_loc);
                result = MAV_RESULT_ACCEPTED;
                gcs().send_text(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                        (double)(new_home_loc.lat*1.0e-7f),
                                        (double)(new_home_loc.lng*1.0e-7f),
                                        (uint32_t)(new_home_loc.alt*0.01f));
            }
            break;
        }

        case MAV_CMD_DO_AUTOTUNE_ENABLE:
            // param1 : enable/disable
            plane.autotune_enable(!is_zero(packet.param1));
            break;

#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            // configure or release parachute
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case PARACHUTE_DISABLE:
                    plane.parachute.enabled(false);
                    break;
                case PARACHUTE_ENABLE:
                    plane.parachute.enabled(true);
                    break;
                case PARACHUTE_RELEASE:
                    // treat as a manual release which performs some additional check of altitude
                    if (plane.parachute.released()) {
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute already released");
                        result = MAV_RESULT_FAILED;
                    } else if (!plane.parachute.enabled()) {
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                        result = MAV_RESULT_FAILED;
                    } else {
                        if (!plane.parachute_manual_release()) {
                            result = MAV_RESULT_FAILED;
                        }
                    }
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
            break;
#endif

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            // param5 : motor count (number of motors to test in sequence)
            result = plane.quadplane.mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4, (uint8_t)packet.param5);
            break;
         case MAV_CMD_DO_SERVO_MOTOR_TEST:
            result = plane.mavlink_servo_motor_test_start(chan, (uint8_t)packet.param1, (uint16_t)packet.param2, packet.param3);
            break;
            
        case MAV_CMD_DO_VTOL_TRANSITION:
            if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_ENGINE_CONTROL:
            if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3)) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_ACCELCAL_VEHICLE_POS:
            result = MAV_RESULT_FAILED;

            if (plane.ins.get_acal()->gcs_vehicle_position(packet.param1)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;
            
        default:
            result = handle_command_long_message(packet);
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, FUNCTOR_BIND(&plane, &Plane::mavlink_set_mode, bool, uint8_t));
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // mark the firmware version in the tlog
        send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(DP_GIT_VERSION)
        send_text(MAV_SEVERITY_INFO, "DEFTWIND: DP_GIT_VERSION# " DP_GIT_VERSION);
#endif
        handle_param_request_list(msg);
        break;
    }

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (plane.g.fence_action != FENCE_ACTION_NONE) {
            send_text(MAV_SEVERITY_WARNING,"Fencing must be disabled");
        } else if (packet.count != plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else if (!check_latlng(packet.lat,packet.lng)) {
            send_text(MAV_SEVERITY_WARNING,"Invalid fence point, lat or lng too large");
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            plane.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (packet.idx >= plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else {
            Vector2l point = plane.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, plane.g.fence_total,
                                             point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &plane.DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(plane.camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != plane.g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        if (hal.rcin->set_overrides(v, 8)) {
            plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
            plane.failsafe.AFS_last_valid_rc_ms =  plane.failsafe.last_valid_rc_ms;
        }

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg->sysid != plane.g.sysid_my_gcs) break;                         // Only accept control from our gcs

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        bool override_active = false;
        int16_t roll = (packet.y == INT16_MAX) ? 0 : plane.channel_roll->get_radio_min() + (plane.channel_roll->get_radio_max() - plane.channel_roll->get_radio_min()) * (packet.y + 1000) / 2000.0f;
        int16_t pitch = (packet.x == INT16_MAX) ? 0 : plane.channel_pitch->get_radio_min() + (plane.channel_pitch->get_radio_max() - plane.channel_pitch->get_radio_min()) * (-packet.x + 1000) / 2000.0f;
        int16_t throttle = (packet.z == INT16_MAX) ? 0 : plane.channel_throttle->get_radio_min() + (plane.channel_throttle->get_radio_max() - plane.channel_throttle->get_radio_min()) * (packet.z) / 1000.0f;
        int16_t yaw = (packet.r == INT16_MAX) ? 0 : plane.channel_rudder->get_radio_min() + (plane.channel_rudder->get_radio_max() - plane.channel_rudder->get_radio_min()) * (packet.r + 1000) / 2000.0f;

        override_active |= hal.rcin->set_override(uint8_t(plane.rcmap.roll() - 1), roll);
        override_active |= hal.rcin->set_override(uint8_t(plane.rcmap.pitch() - 1), pitch);
        override_active |= hal.rcin->set_override(uint8_t(plane.rcmap.throttle() - 1), throttle);
        override_active |= hal.rcin->set_override(uint8_t(plane.rcmap.yaw() - 1), yaw);

        if (override_active) {
            plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
            plane.failsafe.AFS_last_valid_rc_ms =  plane.failsafe.last_valid_rc_ms;
        }
        
        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }
    
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

#if CAMERA == ENABLED
    //deprecated. Use MAV_CMD_DO_DIGICAM_CONFIGURE
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        break;
    }

    //deprecated. Use MAV_CMD_DO_DIGICAM_CONTROL
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        plane.camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    //deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        plane.camera_mount.configure_msg(msg);
        break;
    }

    //deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        plane.camera_mount.control_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, plane.DataFlash, plane.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, plane.gps);
        break;

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        plane.rangefinder.handle_msg(msg);
        break;

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

    case MAVLINK_MSG_ID_LED_CONTROL:
        // send message to Notify
        AP_Notify::handle_led_control(msg);
        break;

    case MAVLINK_MSG_ID_PLAY_TUNE:
        // send message to Notify
        AP_Notify::handle_play_tune(msg);
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
    {
        // Only allow companion computer (or other external controller) to
        // control attitude in GUIDED mode.  We DON'T want external control
        // in e.g., RTL, CICLE. Specifying a single mode for companion
        // computer control is more safe (even more so when using
        // FENCE_ACTION = 4 for geofence failures).
        if (plane.control_mode != GUIDED && plane.control_mode != AVOID_ADSB) { // don't screw up failsafes
            break; 
        }

        mavlink_set_attitude_target_t att_target;
        mavlink_msg_set_attitude_target_decode(msg, &att_target);

        // Mappings: If any of these bits are set, the corresponding input should be ignored.
        // NOTE, when parsing the bits we invert them for easier interpretation but transport has them inverted
        // bit 1: body roll rate
        // bit 2: body pitch rate
        // bit 3: body yaw rate
        // bit 4: unknown
        // bit 5: unknown
        // bit 6: reserved
        // bit 7: throttle
        // bit 8: attitude

        // if not setting all Quaternion values, use _rate flags to indicate which fields.

        // Extract the Euler roll angle from the Quaternion.
        Quaternion q(att_target.q[0], att_target.q[1],
                att_target.q[2], att_target.q[3]);

        // NOTE: att_target.type_mask is inverted for easier interpretation
        att_target.type_mask = att_target.type_mask ^ 0xFF;

        uint8_t attitude_mask = att_target.type_mask & 0b10000111; // q plus rpy

        uint32_t now = AP_HAL::millis();
        if ((attitude_mask & 0b10000001) ||    // partial, including roll
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.x = degrees(q.get_euler_roll()) * 100.0f;

            // Update timer for external roll to the nav control
            plane.guided_state.last_forced_rpy_ms.x = now;
        }

        if ((attitude_mask & 0b10000010) ||    // partial, including pitch
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.y = degrees(q.get_euler_pitch()) * 100.0f;

            // Update timer for external pitch to the nav control
            plane.guided_state.last_forced_rpy_ms.y = now;
        }

        if ((attitude_mask & 0b10000100) ||    // partial, including yaw
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.z = degrees(q.get_euler_yaw()) * 100.0f;

            // Update timer for external yaw to the nav control
            plane.guided_state.last_forced_rpy_ms.z = now;
        }
        if (att_target.type_mask & 0b01000000) { // throttle
            plane.guided_state.forced_throttle = att_target.thrust * 100.0f;

            // Update timer for external throttle
            plane.guided_state.last_forced_throttle_ms = now;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            // don't allow the 0,0 position
            break;
        }
        // sanity check location
        if (!check_latlng(packet.latitude,packet.longitude)) {
            break;
        }
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude / 10;
        plane.ahrs.set_home(new_home_loc);
        plane.home_is_set = HOME_SET_NOT_LOCKED;
        plane.Log_Write_Home_And_Origin();
        gcs().send_home(new_home_loc);
        gcs().send_text(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %um",
                                (double)(new_home_loc.lat*1.0e-7f),
                                (double)(new_home_loc.lng*1.0e-7f),
                                (uint32_t)(new_home_loc.alt*0.01f));
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
    {
        // Only want to allow companion computer position control when
        // in a certain mode to avoid inadvertently sending these
        // kinds of commands when the autopilot is responding to problems
        // in modes such as RTL, CIRCLE, etc.  Specifying ONLY one mode
        // for companion computer control is more safe (provided
        // one uses the FENCE_ACTION = 4 (RTL) for geofence failures).
        if (plane.control_mode != GUIDED && plane.control_mode != AVOID_ADSB) {
            //don't screw up failsafes
            break;
        }

        mavlink_set_position_target_global_int_t pos_target;
        mavlink_msg_set_position_target_global_int_decode(msg, &pos_target);
        // Unexpectedly, the mask is expecting "ones" for dimensions that should
        // be IGNORNED rather than INCLUDED.  See mavlink documentation of the
        // SET_POSITION_TARGET_GLOBAL_INT message, type_mask field.
        const uint16_t alt_mask = 0b1111111111111011; // (z mask at bit 3)
            
        bool msg_valid = true;
        AP_Mission::Mission_Command cmd = {0};
        
        if (pos_target.type_mask & alt_mask)
        {
            cmd.content.location.alt = pos_target.alt * 100;
            cmd.content.location.flags.relative_alt = false;
            cmd.content.location.flags.terrain_alt = false;
            switch (pos_target.coordinate_frame) 
            {
                case MAV_FRAME_GLOBAL_INT:
                    break; //default to MSL altitude
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    cmd.content.location.flags.relative_alt = true;          
                    break;
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    cmd.content.location.flags.relative_alt = true;          
                    cmd.content.location.flags.terrain_alt = true;
                    break;
                default:
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSTION_TARGET_GLOBAL_INT");
                    msg_valid = false;
                    break;
            }    

            if (msg_valid) {
                handle_change_alt_request(cmd);
            }
        } // end if alt_mask       

        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        plane.adsb.handle_message(chan, msg);
        break;

     case MAVLINK_MSG_ID_GCS_GET_PLANE_P900_ID:
        plane.gcs_get_p900_id_flag = true;
        break;
     case MAVLINK_MSG_ID_GCS_SET_PLANE_P900_ID:
        mavlink_gcs_set_plane_p900_id_t need_id;
        mavlink_msg_gcs_set_plane_p900_id_decode(msg, &need_id);
        memset(plane.p900_id, 0, sizeof(plane.p900_id));
        memcpy(plane.p900_id, need_id.id, need_id.count);
        plane.gcs_set_p900_id_flag = true;
        break;

     case MAVLINK_MSG_ID_GCS_SET_PLANE_P900_MODE:
        mavlink_gcs_set_plane_p900_mode_t need_mode;
        mavlink_msg_gcs_set_plane_p900_mode_decode(msg, &need_mode);
        plane.p900_mode = need_mode.type;
        memset(plane.p900_mac, 0, sizeof(plane.p900_mac));
        memcpy(plane.p900_mac, need_mode.mac, need_mode.count);
        plane.gcs_set_p900_mode_flag = true;
        break;

    default:
        handle_common_message(msg);
        break;
    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Plane::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;
    DataFlash.EnableWrites(false);

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs().send_text(MAV_SEVERITY_INFO, "Initialising DeftWind");
    }

    DataFlash.EnableWrites(true);
    in_mavlink_delay = false;
}

/*
 *  send data streams in the given rate range on both links
 */
void Plane::gcs_data_stream_send(void)
{
    gcs().data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Plane::gcs_update(void)
{
    gcs().update();
}

/*
  send airspeed calibration data
 */
void Plane::gcs_send_airspeed_calibration(const Vector3f &vg)
{
    gcs().send_airspeed_calibration(vg);
}

/**
   retry any deferred messages
 */
void Plane::gcs_retry_deferred(void)
{
    gcs().send_message(MSG_RETRY_DEFERRED);
    gcs().service_statustext();
}

/*
  return true if we will accept this packet. Used to implement SYSID_ENFORCE
 */
bool GCS_MAVLINK_Plane::accept_packet(const mavlink_status_t &status, mavlink_message_t &msg)
{
    if (!plane.g2.sysid_enforce) {
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_RADIO || msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }
    return (msg.sysid == plane.g.sysid_my_gcs);
}

Compass *GCS_MAVLINK_Plane::get_compass() const
{
    return &plane.compass;
}

AP_Mission *GCS_MAVLINK_Plane::get_mission()
{
    return &plane.mission;
}

void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    plane.auto_state.next_wp_no_crosstrack = true;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == AUTO && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}

AP_GPS *GCS_MAVLINK_Plane::get_gps() const
{
    return &plane.gps;
}

AP_ServoRelayEvents *GCS_MAVLINK_Plane::get_servorelayevents() const
{
    return &plane.ServoRelayEvents;
}

AP_Rally *GCS_MAVLINK_Plane::get_rally() const
{
    return &plane.rally;
}
