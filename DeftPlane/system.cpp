#include "Plane.h"
#include "version.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Plane::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                         "  logs        log readback/setup mode\n"
                         "  setup       setup mode\n"
                         "  test        test mode\n"
                         "  reboot      reboot to flight mode\n"
                         "\n");
    return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] = {
//   command		function called
//   =======        ===============
    {"logs",        MENU_FUNC(process_logs)},
    {"setup",       MENU_FUNC(setup_mode)},
    {"test",        MENU_FUNC(test_mode)},
    {"reboot",      MENU_FUNC(reboot_board)},
    {"help",        MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Plane::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Plane::run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(nullptr,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(nullptr, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED


static void mavlink_delay_cb_static()
{
    plane.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    plane.failsafe_check();
}

void Plane::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n",
                      (unsigned)hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    //
    load_parameters();

    // initialise stats module
    g2.stats.init();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    set_control_channels();
    
#if HAVE_PX4_MIXER
    if (!quadplane.enable) {
        // this must be before BoardConfig.init() so if
        // BRD_SAFETYENABLE==0 then we don't have safety off yet. For
        // quadplanes we wait till AP_Motors is initialised
        for (uint8_t tries=0; tries<10; tries++) {
            if (setup_failsafe_mixing()) {
                break;
            }
            hal.scheduler->delay(10);
        }
    }
#endif

    gcs().set_dataflash(&DataFlash);

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();
    
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // specify callback function for CLI menu system
#if CLI_ENABLED == ENABLED
    if (g.cli_enabled) {
        gcs().set_run_cli_func(FUNCTOR_BIND_MEMBER(&Plane::run_cli, void, AP_HAL::UARTDriver *));
    }
#endif

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // setup any board specific drivers
    BoardConfig.init();

#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // initialise notify system
    notify.init(false);
    notify_flight_mode(control_mode);

    init_rc_out_main();
    
    // allow servo set on all channels except first 4
    ServoRelayEvents.set_channel_mask(0xFFF0);

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // initialise rangefinder
    init_rangefinder();

    // initialise battery monitoring
    battery.init();

    rpm_sensor.init();

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

    // setup frsky
#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky, and pass a number of parameters to the library
    frsky_telemetry.init(serial_manager, FIRMWARE_STRING,
                         MAV_TYPE_FIXED_WING,
                         &g.fs_batt_voltage, &g.fs_batt_mah);
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    airspeed.init();

    if (g.compass_enabled==true) {
        bool compass_ok = compass.init() && compass.read();
        if (!compass_ok) {
            cliSerial->printf("Compass initialisation failed!\n");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

    // GPS Initialization
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    init_rc_in();               // sets up rc channels from radio

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

#if CLI_ENABLED == ENABLED
    gcs().handle_interactive_setup();
#endif // CLI_ENABLED

    init_capabilities();

    quadplane.setup();

    AP_Param::reload_defaults_file();
    
    startup_ground();

    // don't initialise aux rc output until after quadplane is setup as
    // that can change initial values of channels
    init_rc_out_aux();
    
    // choose the nav controller
    set_nav_controller();

    set_mode((FlightMode)g.initial_mode.get(), MODE_REASON_UNKNOWN);

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();

    // disable safety if requested
    BoardConfig.init_safety();

    printf("\nInit %s Free RAM: %u, Total RAM: %u\n", FIRMWARE_STRING, (unsigned)hal.util->available_memory(), (unsigned)hal.util->total_memory());
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Plane::startup_ground(void)
{
    set_mode(INITIALISING, MODE_REASON_UNKNOWN);

#if (GROUND_START_DELAY > 0)
    gcs().send_text(MAV_SEVERITY_NOTICE,"Ground start with delay");
    delay(GROUND_START_DELAY * 1000);
#else
    gcs().send_text(MAV_SEVERITY_INFO,"Ground start");
#endif

    //INS ground start
    //------------------------
    //
    startup_INS_ground();

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // initialise DataFlash library
    DataFlash.set_mission(&mission);
    DataFlash.setVehicle_Startup_Log_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    failsafe.last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO,"Ground start complete");
}

enum FlightMode Plane::get_previous_mode() {
    return previous_mode; 
}

void Plane::set_mode(enum FlightMode mode, mode_reason_t reason)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }

    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    // perform any cleanup required for prev flight mode
    exit_mode(control_mode);

    // cancel inverted flight
    auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    auto_state.next_wp_no_crosstrack = true;

    // reset landing check
    auto_state.checked_for_autoland = false;
    
    // zero locked course
    steer_state.locked_course_err = 0;

    // reset crash detection
    crash_state.is_crashed = false;
    crash_state.impact_detected = false;

    // reset external attitude guidance
    guided_state.last_forced_rpy_ms.zero();
    guided_state.last_forced_throttle_ms = 0;

    // set mode
    previous_mode = control_mode;
    control_mode = mode;
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.update_control_mode(control_mode);
#endif
    
    if (previous_mode == AUTOTUNE && control_mode != AUTOTUNE) {
        // restore last gains
        autotune_restore();
    }

    // zero initial pitch and highest airspeed on mode change
    auto_state.highest_airspeed = 0;
    auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    prev_WP_loc = current_loc;

    // new mode means new loiter
    loiter.start_time_ms = 0;

    // record time of mode change
    last_mode_change_ms = AP_HAL::millis();
    
    // assume non-VTOL mode
    auto_state.vtol_mode = false;
    auto_state.vtol_loiter = false;
    
    switch(control_mode)
    {
    case INITIALISING:
        auto_throttle_mode = true;
        auto_navigation_mode = false;
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        break;

    case AUTOTUNE:
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        autotune_start();
        break;

    case ACRO:
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;
        
    case CRUISE:
        auto_throttle_mode = true;
        auto_navigation_mode = false;
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;
        
        // for ArduSoar soaring_controller
        g2.soaring_controller.init_cruising();
        
        set_target_altitude_current();
        break;

    case FLY_BY_WIRE_B:
        auto_throttle_mode = true;
        auto_navigation_mode = false;
        
        // for ArduSoar soaring_controller
        g2.soaring_controller.init_cruising();

        set_target_altitude_current();
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        next_WP_loc.alt = current_loc.alt;
        break;
    case NO_GPS_RTL:
         // the altitude to circle at is taken from the current altitude
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        next_WP_loc.alt = current_loc.alt;
        Location cur_loc;
        ahrs.get_position(cur_loc);
        distance_to_home = get_distance(cur_loc, home);
        heading_to_home = get_bearing_cd(cur_loc, home)*0.01;
        if(distance_to_home < 300){
            no_gps_rtl_home_flag = NO_GPS_RTL_NEAR_HOME;
            emergency_return = true;
        }else{
            no_gps_rtl_home_flag = NO_GPS_RTL_NONE;
        }
        gcs().send_text(MAV_SEVERITY_CRITICAL, "NO_GPS_RTL flag:%d, distanc:%4.2lf, heading:%3.2lf",
                                            no_gps_rtl_home_flag, (double)distance_to_home, (double)heading_to_home);
        break;

    case AUTO:
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        if (quadplane.available() && quadplane.enable == 2) {
            auto_state.vtol_mode = true;
        } else {
            auto_state.vtol_mode = false;
        }
        next_WP_loc = prev_WP_loc = current_loc;
        // start or resume the mission, based on MIS_AUTORESET
        mission.start_or_resume();
		
        g2.soaring_controller.init_cruising();
        break;

    case RTL:
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        prev_WP_loc = current_loc;
        do_RTL(get_RTL_altitude());
        break;

    case LOITER:
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        do_loiter_at_location();
		
        if (g2.soaring_controller.is_active() &&
            g2.soaring_controller.suppress_throttle()) {
			g2.soaring_controller.init_thermalling();
			g2.soaring_controller.get_target(next_WP_loc); // ahead on flight path
		}
		
        break;

    case AVOID_ADSB:
    case GUIDED:
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        guided_throttle_passthru = false;
        /*
          when entering guided mode we set the target as the current
          location. This matches the behaviour of the copter code
        */
        guided_WP_loc = current_loc;
        set_guided_WP();
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
        auto_navigation_mode = false;
        if (!quadplane.init_mode()) {
            control_mode = previous_mode;
        } else {
            auto_throttle_mode = false;
            auto_state.vtol_mode = true;
        }
        break;
    }

    // start with throttle suppressed in auto_throttle modes
    throttle_suppressed = auto_throttle_mode;

    adsb.set_is_auto_mode(auto_navigation_mode);

    if (should_log(MASK_LOG_MODE))
        DataFlash.Log_Write_Mode(control_mode);

    // update notify with flight mode change
    notify_flight_mode(control_mode);

    // reset steering integrator on mode change
    steerController.reset_I();    
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Plane::mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case CIRCLE:
    case STABILIZE:
    case TRAINING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case AVOID_ADSB:
    case GUIDED:
    case AUTO:
    case RTL:
    case LOITER:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
    case NO_GPS_RTL:
        set_mode((enum FlightMode)mode, MODE_REASON_GCS_COMMAND);
        return true;
    }
    return false;
}

// exit_mode - perform any cleanup required when leaving a flight mode
void Plane::exit_mode(enum FlightMode mode)
{
    // stop mission when we leave auto
    if (mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();

            if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
                !quadplane.is_vtol_land(mission.get_current_nav_cmd().id))
            {
                landing.restart_landing_sequence();
            }
        }
        auto_state.started_flying_in_auto_ms = 0;
    }
}

void Plane::check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if (failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        if (!auto_throttle_mode && failsafe.state == FAILSAFE_SHORT &&
                   (tnow - failsafe.ch3_timer_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_LONG, MODE_REASON_RADIO_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == AUTO &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        } else if (!emergency_return && flight_stage == AP_Vehicle::FixedWing::FLIGHT_NORMAL && arming.is_armed() &&
                    g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs().chan(0).last_radio_status_remrssi_ms != 0 &&
                   (tnow - gcs().chan(0).last_radio_status_remrssi_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        }
    } else {
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - failsafe.last_heartbeat_ms) < g.short_fs_timeout*1000) {
            failsafe.state = FAILSAFE_NONE;
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.ch3_failsafe) {
            failsafe.state = FAILSAFE_NONE;
        }
    }
}

void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if(failsafe.state == FAILSAFE_NONE && flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // The condition is checked and the flag ch3_failsafe is set in radio.cpp
        if(failsafe.ch3_failsafe) {
            failsafe_short_on_event(FAILSAFE_SHORT, MODE_REASON_RADIO_FAILSAFE);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.ch3_failsafe) {
            failsafe_short_off_event(MODE_REASON_RADIO_FAILSAFE);
        }
    }
}


void Plane::startup_INS_ground(void)
{
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
        hal.scheduler->delay(100);
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed(true);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING,"No airspeed");
    }
}

// updates the status of the notify objects
// should be called at 50hz
void Plane::update_notify()
{
    notify.update();
}

void Plane::resetPerfData(void) 
{
    perf.mainLoop_count = 0;
    perf.G_Dt_max       = 0;
    perf.G_Dt_min       = 0;
    perf.num_long       = 0;
    perf.start_ms       = millis();
    perf.last_log_dropped = DataFlash.num_dropped();
}


void Plane::print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->printf("Manual");
        break;
    case CIRCLE:
        port->printf("Circle");
        break;
    case NO_GPS_RTL:
        port->printf("NO_GPS_RTL");
        break;
    case STABILIZE:
        port->printf("Stabilize");
        break;
    case TRAINING:
        port->printf("Training");
        break;
    case ACRO:
        port->printf("ACRO");
        break;
    case FLY_BY_WIRE_A:
        port->printf("FBW_A");
        break;
    case AUTOTUNE:
        port->printf("AUTOTUNE");
        break;
    case FLY_BY_WIRE_B:
        port->printf("FBW_B");
        break;
    case CRUISE:
        port->printf("CRUISE");
        break;
    case AUTO:
        port->printf("AUTO");
        break;
    case RTL:
        port->printf("RTL");
        break;
    case LOITER:
        port->printf("Loiter");
        break;
    case AVOID_ADSB:
        port->printf("AVOID_ADSB");
        break;
    case GUIDED:
        port->printf("Guided");
        break;
    case QSTABILIZE:
        port->printf("QStabilize");
        break;
    case QHOVER:
        port->printf("QHover");
        break;
    case QLOITER:
        port->printf("QLoiter");
        break;
    case QLAND:
        port->printf("QLand");
        break;
    case QRTL:
        port->printf("QRTL");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}

// sets notify object flight mode information
void Plane::notify_flight_mode(enum FlightMode mode)
{
    AP_Notify::flags.flight_mode = mode;

    // set flight mode string
    switch (mode) {
    case MANUAL:
        notify.set_flight_mode_str("MANU");
        break;
    case CIRCLE:
        notify.set_flight_mode_str("CIRC");
        break;
    case STABILIZE:
        notify.set_flight_mode_str("STAB");
        break;
    case TRAINING:
        notify.set_flight_mode_str("TRAN");
        break;
    case ACRO:
        notify.set_flight_mode_str("ACRO");
        break;
    case FLY_BY_WIRE_A:
        notify.set_flight_mode_str("FBWA");
        break;
    case FLY_BY_WIRE_B:
        notify.set_flight_mode_str("FBWB");
        break;
    case CRUISE:
        notify.set_flight_mode_str("CRUS");
        break;
    case AUTOTUNE:
        notify.set_flight_mode_str("ATUN");
        break;
    case AUTO:
        notify.set_flight_mode_str("AUTO");
        break;
    case RTL:
        notify.set_flight_mode_str("RTL ");
        break;
    case LOITER:
        notify.set_flight_mode_str("LOITER");
        break;
    case AVOID_ADSB:
        notify.set_flight_mode_str("AVOI");
        break;
    case GUIDED:
        notify.set_flight_mode_str("GUID");
        break;
    case INITIALISING:
        notify.set_flight_mode_str("INIT");
        break;
    case QSTABILIZE:
        notify.set_flight_mode_str("QSTB");
        break;
    case QHOVER:
        notify.set_flight_mode_str("QHOV");
        break;
    case QLOITER:
        notify.set_flight_mode_str("QLOT");
        break;
    case QLAND:
        notify.set_flight_mode_str("QLND");
        break;
    case QRTL:
        notify.set_flight_mode_str("QRTL");
        break;
    case NO_GPS_RTL:
        notify.set_flight_mode_str("NOGPSRTL");
    default:
        notify.set_flight_mode_str("----");
        break;
    }
}

#if CLI_ENABLED == ENABLED
void Plane::print_comma(void)
{
    cliSerial->printf(",");
}
#endif

/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    return DataFlash.should_log(mask);
#else
    return false;
#endif
}

/*
  return throttle percentage from 0 to 100 for normal use and -100 to 100 when using reverse thrust
 */
int8_t Plane::throttle_percentage(void)
{
    if (quadplane.in_vtol_mode()) {
        return quadplane.throttle_percentage();
    }
    // to get the real throttle we need to use norm_output() which
    // returns a number from -1 to 1.
    float throttle = SRV_Channels::get_output_norm(SRV_Channel::k_throttle);
    if (aparm.throttle_min >= 0) {
        return constrain_int16(50*(throttle+1), 0, 100);
    } else {
        // reverse thrust
        return constrain_int16(100*throttle, -100, 100);
    }
}

/*
  update AHRS soft arm state and log as needed
 */
void Plane::change_arm_state(void)
{
    Log_Arm_Disarm();
    update_soft_armed();
    quadplane.set_armed(hal.util->get_soft_armed());
}

/*
  arm motors
 */
bool Plane::arm_motors(AP_Arming::ArmingMethod method)
{
    if (!arming.arm(method)) {
        return false;
    }

    change_arm_state();
    return true;
}

/*
  disarm motors
 */
bool Plane::disarm_motors(void)
{
    float alt = 0.0f;
    ahrs.get_relative_position_D_home(alt);

    if(-alt > g.disarm_hight){
        gcs().send_text(MAV_SEVERITY_INFO,"The current height %4.2lfm accepts a disarm common.", (double)-alt);
        return false;
    }
    if (!arming.disarm()) {
        return false;
    }
    if (control_mode != AUTO) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    throttle_suppressed = auto_throttle_mode;
    
    //only log if disarming was successful
    change_arm_state();

    // reload target airspeed which could have been modified by a mission
    plane.aparm.airspeed_cruise_cm.load();
    
    return true;
}
