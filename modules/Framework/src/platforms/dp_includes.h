/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_includes.h
 *
 * Includes headers depending on the build target
 */

#pragma once

#include <stdbool.h>
#if 0
#if defined(__DP_NUTTX)
/*
 * Building for NuttX
 */
#include <nuttx/config.h>
#include <uORB/uORB.h>
#ifdef __cplusplus
#include <platforms/nuttx/dp_messages/dp_rc_channels.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_attitude_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_manual_control_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_actuator_controls.h>
#include <platforms/nuttx/dp_messages/dp_actuator_controls_0.h>
#include <platforms/nuttx/dp_messages/dp_actuator_controls_1.h>
#include <platforms/nuttx/dp_messages/dp_actuator_controls_2.h>
#include <platforms/nuttx/dp_messages/dp_actuator_controls_3.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_rates_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_attitude.h>
#include <platforms/nuttx/dp_messages/dp_control_state.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_control_mode.h>
#include <platforms/nuttx/dp_messages/dp_actuator_armed.h>
#include <platforms/nuttx/dp_messages/dp_parameter_update.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_status.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_local_position_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_global_velocity_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_local_position.h>
#include <platforms/nuttx/dp_messages/dp_position_setpoint_triplet.h>
#include <platforms/nuttx/dp_messages/dp_offboard_control_mode.h>
#include <platforms/nuttx/dp_messages/dp_vehicle_force_setpoint.h>
#include <platforms/nuttx/dp_messages/dp_camera_trigger.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#elif defined(__DP_POSIX)
#include <string.h>
#include <assert.h>
#include <uORB/uORB.h>

#define ASSERT(x) assert(x)

#ifdef __cplusplus
#include <platforms/posix/dp_messages/dp_rc_channels.h>
#include <platforms/posix/dp_messages/dp_vehicle_attitude_setpoint.h>
#include <platforms/posix/dp_messages/dp_manual_control_setpoint.h>
#include <platforms/posix/dp_messages/dp_actuator_controls.h>
#include <platforms/posix/dp_messages/dp_actuator_controls_0.h>
#include <platforms/posix/dp_messages/dp_actuator_controls_1.h>
#include <platforms/posix/dp_messages/dp_actuator_controls_2.h>
#include <platforms/posix/dp_messages/dp_actuator_controls_3.h>
#include <platforms/posix/dp_messages/dp_vehicle_rates_setpoint.h>
#include <platforms/posix/dp_messages/dp_vehicle_attitude.h>
#include <platforms/posix/dp_messages/dp_control_state.h>
#include <platforms/posix/dp_messages/dp_vehicle_control_mode.h>
#include <platforms/posix/dp_messages/dp_actuator_armed.h>
#include <platforms/posix/dp_messages/dp_parameter_update.h>
#include <platforms/posix/dp_messages/dp_vehicle_status.h>
#include <platforms/posix/dp_messages/dp_vehicle_local_position_setpoint.h>
#include <platforms/posix/dp_messages/dp_vehicle_global_velocity_setpoint.h>
#include <platforms/posix/dp_messages/dp_vehicle_local_position.h>
#include <platforms/posix/dp_messages/dp_position_setpoint_triplet.h>
#endif
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#else
#error "No target platform defined"
#endif
#endif
