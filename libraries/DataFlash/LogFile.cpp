#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <GCS_MAVLink/GCS.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "DataFlash.h"
#include "DataFlash_File.h"
#include "DataFlash_MAVLink.h"
#include "DFMessageWriter.h"

extern const AP_HAL::HAL& hal;


/*
  read and print a log entry using the format strings from the given structure
 */
void DataFlash_Backend::_print_log_entry(uint8_t msg_type,
                                         print_mode_fn print_mode,
                                         AP_HAL::BetterStream *port)
{
    uint8_t i;
    for (i=0; i<num_types(); i++) {
        if (msg_type == structure(i)->msg_type) {
            break;
        }
    }
    if (i == num_types()) {
        port->printf("UNKN, %u\n", (unsigned)msg_type);
        return;
    }
    const struct LogStructure *log_structure = structure(i);
    uint8_t msg_len = log_structure->msg_len - 3;
    uint8_t pkt[msg_len];
    if (!ReadBlock(pkt, msg_len)) {
        return;
    }
    port->printf("%s, ", log_structure->name);
    for (uint8_t ofs=0, fmt_ofs=0; ofs<msg_len; fmt_ofs++) {
        char fmt = log_structure->format[fmt_ofs];
        switch (fmt) {
        case 'b': {
            port->printf("%d", (int)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'B': {
            port->printf("%u", (unsigned)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'h': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%d", (int)v);
            ofs += sizeof(v);
            break;
        }
        case 'H': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%u", (unsigned)v);
            ofs += sizeof(v);
            break;
        }
        case 'i': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%ld", (long)v);
            ofs += sizeof(v);
            break;
        }
        case 'I': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%lu", (unsigned long)v);
            ofs += sizeof(v);
            break;
        }
        case 'q': {
            int64_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%lld", (long long)v);
            ofs += sizeof(v);
            break;
        }
        case 'Q': {
            uint64_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%llu", (unsigned long long)v);
            ofs += sizeof(v);
            break;
        }
        case 'f': {
            float v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%f", (double)v);
            ofs += sizeof(v);
            break;
        }
        case 'd': {
            double v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            // note that %f here *really* means a single-precision
            // float, so we lose precision printing this double out
            // dtoa_engine needed....
            port->printf("%f", (double)v);
            ofs += sizeof(v);
            break;
        }
        case 'c': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'C': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'e': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'E': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'L': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            print_latlon(port, v);
            ofs += sizeof(v);
            break;
        }
        case 'n': {
            char v[5];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'N': {
            char v[17];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'Z': {
            char v[65];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'M': {
            print_mode(port, pkt[ofs]);
            ofs += 1;
            break;
        }
        default:
            ofs = msg_len;
            break;
        }
        if (ofs < msg_len) {
            port->printf(", ");
        }
    }
    port->printf("\n");
}

/*
  write a structure format to the log - should be in frontend
 */
void DataFlash_Backend::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = s->msg_type;
    pkt.length = s->msg_len;
    strncpy(pkt.name, s->name, sizeof(pkt.name));
    strncpy(pkt.format, s->format, sizeof(pkt.format));
    strncpy(pkt.labels, s->labels, sizeof(pkt.labels));
}

/*
  write a structure format to the log
 */
bool DataFlash_Backend::Log_Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Log_Fill_Format(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool DataFlash_Backend::Log_Write_Parameter(const char *name, float value)
{
    struct log_Parameter pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
        time_us : AP_HAL::micros64(),
        name  : {},
        value : value
    };
    strncpy(pkt.name, name, sizeof(pkt.name));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool DataFlash_Backend::Log_Write_Parameter(const AP_Param *ap,
                                            const AP_Param::ParamToken &token,
                                            enum ap_var_type type)
{
    char name[16];
    ap->copy_name_token(token, &name[0], sizeof(name), true);
    return Log_Write_Parameter(name, ap->cast_to_float(type));
}

// Write an GPS packet
void DataFlash_Class::Log_Write_GPS(const AP_GPS &gps, uint8_t i, uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const struct Location &loc = gps.location(i);
    //0x10: use; 0x00: not use
    uint8_t use = gps.get_gps_type(i) + (gps.primary_sensor() == i ? 16:0);
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPS_MSG+i)),
        time_us       : time_us,
        status        : (uint8_t)gps.status(i),
        gps_week_ms   : gps.time_week_ms(i),
        gps_week      : gps.time_week(i),
        num_sats      : gps.num_sats(i),
        hdop          : gps.get_hdop(i),
        latitude      : loc.lat,
        longitude     : loc.lng,
        altitude      : loc.alt,
        ground_speed  : gps.ground_speed(i),
        ground_course : gps.ground_course(i),
        vel_z         : gps.velocity(i).z,
        used          : use
    };
    WriteBlock(&pkt, sizeof(pkt));

    /* write auxiliary accuracy information as well */
    float hacc = 0, vacc = 0, sacc = 0, hdgacc = 0;
    gps.horizontal_accuracy(i, hacc);
    gps.vertical_accuracy(i, vacc);
    gps.speed_accuracy(i, sacc);
    gps.heading_accuracy(i, hdgacc);
    struct log_GPA pkt2 = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPA_MSG+i)),
        time_us       : time_us,
        vdop          : gps.get_vdop(i),
        hacc          : (uint16_t)MIN((hacc*100), UINT16_MAX),
        vacc          : (uint16_t)MIN((vacc*100), UINT16_MAX),
        sacc          : (uint16_t)MIN((sacc*100), UINT16_MAX),
        have_vv       : (uint8_t)gps.have_vertical_velocity(i),
        sample_ms     : gps.last_message_time_ms(i),
        hdgs          : gps.heading_status(i),
        hdg           : gps.get_heading(i),
        hdgacc        : (uint16_t)(hdgacc*100)
    };
    WriteBlock(&pkt2, sizeof(pkt2));
}


// Write an RFND (rangefinder) packet
void DataFlash_Class::Log_Write_RFND(const RangeFinder &rangefinder)
{
    struct log_RFND pkt = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_RFND_MSG)),
        time_us       : AP_HAL::micros64(),
        dist1         : rangefinder.distance_cm(0),
        orient1       : rangefinder.get_orientation(0),
        dist2         : rangefinder.distance_cm(1),
        orient2       : rangefinder.get_orientation(1)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RCIN packet
void DataFlash_Class::Log_Write_RCIN(void)
{
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : hal.rcin->read(0),
        chan2         : hal.rcin->read(1),
        chan3         : hal.rcin->read(2),
        chan4         : hal.rcin->read(3),
        chan5         : hal.rcin->read(4),
        chan6         : hal.rcin->read(5),
        chan7         : hal.rcin->read(6),
        chan8         : hal.rcin->read(7),
        chan9         : hal.rcin->read(8),
        chan10        : hal.rcin->read(9),
        chan11        : hal.rcin->read(10),
        chan12        : hal.rcin->read(11),
        chan13        : hal.rcin->read(12),
        chan14        : hal.rcin->read(13)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an SERVO packet
void DataFlash_Class::Log_Write_RCOUT(void)
{
    struct log_RCOUT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : hal.rcout->read(0),
        chan2         : hal.rcout->read(1),
        chan3         : hal.rcout->read(2),
        chan4         : hal.rcout->read(3),
        chan5         : hal.rcout->read(4),
        chan6         : hal.rcout->read(5),
        chan7         : hal.rcout->read(6),
        chan8         : hal.rcout->read(7),
        chan9         : hal.rcout->read(8),
        chan10        : hal.rcout->read(9),
        chan11        : hal.rcout->read(10),
        chan12        : hal.rcout->read(11),
        chan13        : hal.rcout->read(12),
        chan14        : hal.rcout->read(13)
    };
    WriteBlock(&pkt, sizeof(pkt));
    Log_Write_ESC();
}

// Write an RSSI packet
void DataFlash_Class::Log_Write_RSSI(AP_RSSI &rssi)
{
    struct log_RSSI pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RSSI_MSG),
        time_us       : AP_HAL::micros64(),
        RXRSSI        : rssi.read_receiver_rssi()
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
void DataFlash_Class::Log_Write_Baro(AP_Baro &baro, uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    float climbrate = baro.get_climb_rate();
    float drift_offset = baro.get_baro_drift_offset();
    float ground_temp = baro.get_ground_temperature();
    struct log_BARO pkt = {
        LOG_PACKET_HEADER_INIT(LOG_BARO_MSG),
        time_us       : time_us,
        altitude      : baro.get_altitude(0),
        pressure      : baro.get_pressure(0),
        temperature   : (int16_t)(baro.get_temperature(0) * 100 + 0.5f),
        climbrate     : climbrate,
        sample_time_ms: baro.get_last_update(0),
        drift_offset  : drift_offset,
        ground_temp   : ground_temp,
    };
    WriteBlock(&pkt, sizeof(pkt));

    if (baro.num_instances() > 1 && baro.healthy(1)) {
        struct log_BARO pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_BAR2_MSG),
            time_us       : time_us,
            altitude      : baro.get_altitude(1),
            pressure      : baro.get_pressure(1),
            temperature   : (int16_t)(baro.get_temperature(1) * 100 + 0.5f),
            climbrate     : climbrate,
            sample_time_ms: baro.get_last_update(1),
            drift_offset  : drift_offset,
            ground_temp   : ground_temp,
        };
        WriteBlock(&pkt2, sizeof(pkt2));
    }

    if (baro.num_instances() > 2 && baro.healthy(2)) {
        struct log_BARO pkt3 = {
            LOG_PACKET_HEADER_INIT(LOG_BAR3_MSG),
            time_us       : time_us,
            altitude      : baro.get_altitude(2),
            pressure      : baro.get_pressure(2),
            temperature   : (int16_t)(baro.get_temperature(2) * 100 + 0.5f),
            climbrate     : climbrate,
            sample_time_ms: baro.get_last_update(2),
            drift_offset  : drift_offset,
            ground_temp   : ground_temp,
        };
        WriteBlock(&pkt3, sizeof(pkt3));
    }
}

// Write an raw accel/gyro data packet
void DataFlash_Class::Log_Write_IMU(const AP_InertialSensor &ins)
{
    uint64_t time_us = AP_HAL::micros64();
    const Vector3f &gyro = ins.get_gyro(0);
    const Vector3f &accel = ins.get_accel(0);
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        time_us : time_us,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z,
        gyro_error  : ins.get_gyro_error_count(0),
        accel_error : ins.get_accel_error_count(0),
        temperature : ins.get_temperature(0),
        gyro_health : (uint8_t)ins.get_gyro_health(0),
        accel_health : (uint8_t)ins.get_accel_health(0),
        gyro_rate : ins.get_gyro_rate_hz(0),
        accel_rate : ins.get_accel_rate_hz(0),
    };
    WriteBlock(&pkt, sizeof(pkt));
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }

    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &accel2 = ins.get_accel(1);
    struct log_IMU pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_IMU2_MSG),
        time_us : time_us,
        gyro_x  : gyro2.x,
        gyro_y  : gyro2.y,
        gyro_z  : gyro2.z,
        accel_x : accel2.x,
        accel_y : accel2.y,
        accel_z : accel2.z,
        gyro_error  : ins.get_gyro_error_count(1),
        accel_error : ins.get_accel_error_count(1),
        temperature : ins.get_temperature(1),
        gyro_health : (uint8_t)ins.get_gyro_health(1),
        accel_health : (uint8_t)ins.get_accel_health(1),
        gyro_rate : ins.get_gyro_rate_hz(1),
        accel_rate : ins.get_accel_rate_hz(1),
    };
    WriteBlock(&pkt2, sizeof(pkt2));
    if (ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) {
        return;
    }
    const Vector3f &gyro3 = ins.get_gyro(2);
    const Vector3f &accel3 = ins.get_accel(2);
    struct log_IMU pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_IMU3_MSG),
        time_us : time_us,
        gyro_x  : gyro3.x,
        gyro_y  : gyro3.y,
        gyro_z  : gyro3.z,
        accel_x : accel3.x,
        accel_y : accel3.y,
        accel_z : accel3.z,
        gyro_error  : ins.get_gyro_error_count(2),
        accel_error : ins.get_accel_error_count(2),
        temperature : ins.get_temperature(2),
        gyro_health : (uint8_t)ins.get_gyro_health(2),
        accel_health : (uint8_t)ins.get_accel_health(2),
        gyro_rate : ins.get_gyro_rate_hz(2),
        accel_rate : ins.get_accel_rate_hz(2),
    };
    WriteBlock(&pkt3, sizeof(pkt3));
}

// Write an accel/gyro delta time data packet
void DataFlash_Class::Log_Write_IMUDT(const AP_InertialSensor &ins, uint64_t time_us, uint8_t imu_mask)
{
    float delta_t = ins.get_delta_time();
    float delta_vel_t = ins.get_delta_velocity_dt(0);
    float delta_ang_t = ins.get_delta_angle_dt(0);
    Vector3f delta_angle, delta_velocity;
    ins.get_delta_angle(0, delta_angle);
    ins.get_delta_velocity(0, delta_velocity);

    struct log_IMUDT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_IMUDT_MSG),
        time_us : time_us,
        delta_time   : delta_t,
        delta_vel_dt : delta_vel_t,
        delta_ang_dt : delta_ang_t,
        delta_ang_x  : delta_angle.x,
        delta_ang_y  : delta_angle.y,
        delta_ang_z  : delta_angle.z,
        delta_vel_x  : delta_velocity.x,
        delta_vel_y  : delta_velocity.y,
        delta_vel_z  : delta_velocity.z
    };
    if (imu_mask & 1) {
        WriteBlock(&pkt, sizeof(pkt));
    }
    if ((ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) || !ins.use_gyro(1)) {
        return;
    }

    delta_vel_t = ins.get_delta_velocity_dt(1);
    delta_ang_t = ins.get_delta_angle_dt(1);
    if (!ins.get_delta_angle(1, delta_angle)) {
        delta_angle.zero();
    }
    if (!ins.get_delta_velocity(1, delta_velocity)) {
        delta_velocity.zero();
    }
    struct log_IMUDT pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_IMUDT2_MSG),
        time_us     : time_us,
        delta_time   : delta_t,
        delta_vel_dt : delta_vel_t,
        delta_ang_dt : delta_ang_t,
        delta_ang_x  : delta_angle.x,
        delta_ang_y  : delta_angle.y,
        delta_ang_z  : delta_angle.z,
        delta_vel_x  : delta_velocity.x,
        delta_vel_y  : delta_velocity.y,
        delta_vel_z  : delta_velocity.z
    };
    if (imu_mask & 2) {
        WriteBlock(&pkt2, sizeof(pkt2));
    }

    if ((ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) || !ins.use_gyro(2)) {
        return;
    }
    delta_vel_t = ins.get_delta_velocity_dt(1);
    delta_ang_t = ins.get_delta_angle_dt(2);
    if (!ins.get_delta_angle(2, delta_angle)) {
        delta_angle.zero();
    }
    if (!ins.get_delta_velocity(2, delta_velocity)) {
        delta_velocity.zero();
    }
    struct log_IMUDT pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_IMUDT3_MSG),
        time_us     : time_us,
        delta_time   : delta_t,
        delta_vel_dt : delta_vel_t,
        delta_ang_dt : delta_ang_t,
        delta_ang_x  : delta_angle.x,
        delta_ang_y  : delta_angle.y,
        delta_ang_z  : delta_angle.z,
        delta_vel_x  : delta_velocity.x,
        delta_vel_y  : delta_velocity.y,
        delta_vel_z  : delta_velocity.z
    };
    if (imu_mask & 4) {
        WriteBlock(&pkt3, sizeof(pkt3));
    }
}

void DataFlash_Class::Log_Write_Vibration(const AP_InertialSensor &ins)
{
    uint64_t time_us = AP_HAL::micros64();
    Vector3f vibration = ins.get_vibration_levels();
    struct log_Vibe pkt = {
        LOG_PACKET_HEADER_INIT(LOG_VIBE_MSG),
        time_us     : time_us,
        vibe_x      : vibration.x,
        vibe_y      : vibration.y,
        vibe_z      : vibration.z,
        clipping_0  : ins.get_accel_clip_count(0),
        clipping_1  : ins.get_accel_clip_count(1),
        clipping_2  : ins.get_accel_clip_count(2)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a mission command. Total length : 36 bytes
bool DataFlash_Backend::Log_Write_Mission_Cmd(const AP_Mission &mission,
                                              const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    return Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}

void DataFlash_Backend::Log_Write_EntireMission(const AP_Mission &mission)
{
    DFMessageWriter_WriteEntireMission writer;
    writer.set_dataflash_backend(this);
    writer.set_mission(&mission);
    writer.process();
}

// Write a text message to the log
bool DataFlash_Backend::Log_Write_Message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };
    strncpy(pkt.msg, message, sizeof(pkt.msg));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

void DataFlash_Class::Log_Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    struct log_POWR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        time_us : AP_HAL::micros64(),
        Vcc     : hal.analogin->board_voltage(),
        Vservo  : hal.analogin->servorail_voltage(),
        flags   : hal.analogin->power_status_flags()
    };
    WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write an AHRS2 packet
void DataFlash_Class::Log_Write_AHRS2(AP_AHRS &ahrs)
{
    Vector3f euler;
    struct Location loc;
    Quaternion quat;
    if (!ahrs.get_secondary_attitude(euler) || !ahrs.get_secondary_position(loc)) {
        return;
    }
    ahrs.get_secondary_quaternion(quat);
    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AHR2_MSG),
        time_us : AP_HAL::micros64(),
        roll  : (int16_t)(degrees(euler.x)*100),
        pitch : (int16_t)(degrees(euler.y)*100),
        yaw   : (uint16_t)(wrap_360_cd(degrees(euler.z)*100)),
        alt   : loc.alt*1.0e-2f,
        lat   : loc.lat,
        lng   : loc.lng,
        q1    : quat.q1,
        q2    : quat.q2,
        q3    : quat.q3,
        q4    : quat.q4,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a POS packet
void DataFlash_Class::Log_Write_POS(AP_AHRS &ahrs)
{
    Location loc;
    if (!ahrs.get_position(loc)) {
        return;
    }
    float home, origin;
    ahrs.get_relative_position_D_home(home);
    struct log_POS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POS_MSG),
        time_us        : AP_HAL::micros64(),
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt*1.0e-2f,
        rel_home_alt   : -home,
        rel_origin_alt : ahrs.get_relative_position_D_origin(origin) ? -origin : nanf("ARDUPILOT")
    };
    WriteBlock(&pkt, sizeof(pkt));
}

#if AP_AHRS_NAVEKF_AVAILABLE
void DataFlash_Class::Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled)
{
    // only log EKF2 if enabled
    if (ahrs.get_NavEKF2().activeCores() > 0) {
        Log_Write_EKF2(ahrs, optFlowEnabled);
    }
    // only log EKF3 if enabled
    if (ahrs.get_NavEKF3().activeCores() > 0) {
        Log_Write_EKF3(ahrs, optFlowEnabled);
    }
}


/*
  write an EKF timing message
 */
void DataFlash_Class::Log_Write_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing)
{
    Log_Write(name,
              "TimeUS,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VelMin,VelMax", "QIffffffff",
              time_us,
              timing.count,
              (double)timing.dtIMUavg_min,
              (double)timing.dtIMUavg_max,
              (double)timing.dtEKFavg_min,
              (double)timing.dtEKFavg_max,
              (double)timing.delAngDT_min,
              (double)timing.delAngDT_max,
              (double)timing.delVelDT_min,
              (double)timing.delVelDT_max);
}

void DataFlash_Class::Log_Write_EKF2(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled)
{
    uint64_t time_us = AP_HAL::micros64();
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f dAngBias;
    Vector3f dVelBias;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF2().getEulerAngles(0,euler);
    ahrs.get_NavEKF2().getVelNED(0,velNED);
    ahrs.get_NavEKF2().getPosNE(0,posNE);
    ahrs.get_NavEKF2().getPosD(0,posD);
    ahrs.get_NavEKF2().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF2().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NKF1_MSG),
        time_us : time_us,
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    uint8_t magIndex = ahrs.get_NavEKF2().getActiveMag(0);
    ahrs.get_NavEKF2().getAccelZBias(0,azbias);
    ahrs.get_NavEKF2().getWind(0,wind);
    ahrs.get_NavEKF2().getMagNED(0,magNED);
    ahrs.get_NavEKF2().getMagXYZ(0,magXYZ);
    ahrs.get_NavEKF2().getGyroScaleErrorPercentage(0,gyroScaleFactor);
    struct log_NKF2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF2_MSG),
        time_us : time_us,
        AZbias  : (int8_t)(100*azbias),
        scaleX  : (int16_t)(100*gyroScaleFactor.x),
        scaleY  : (int16_t)(100*gyroScaleFactor.y),
        scaleZ  : (int16_t)(100*gyroScaleFactor.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        index   : (uint8_t)(magIndex)
    };
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF2().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF3_MSG),
        time_us : time_us,
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov)
    };
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF2().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF2().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF2().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF2().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF2().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF2().getTiltError(0,tiltError);
    int8_t primaryIndex = ahrs.get_NavEKF2().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF4_MSG),
        time_us : time_us,
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint16_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF2().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF2().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF5_MSG),
        time_us : time_us,
        normInnov : (uint8_t)(MIN(100*normInnov,255)),
        FIX : (int16_t)(1000*flowInnovX),
        FIY : (int16_t)(1000*flowInnovY),
        AFI : (int16_t)(1000*auxFlowInnov),
        HAGL : (int16_t)(100*HAGL),
        offset : (int16_t)(100*gndOffset),
        RI : (int16_t)(100*rngInnov),
        meaRng : (uint16_t)(100*range),
        errHAGL : (uint16_t)(100*gndOffsetErr),
        angErr : (float)predictorErrors.x,
        velErr : (float)predictorErrors.y,
        posErr : (float)predictorErrors.z
     };
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF2().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {
        LOG_PACKET_HEADER_INIT(LOG_NKQ1_MSG),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    WriteBlock(&pktq1, sizeof(pktq1));

    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF2().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF2().getEulerAngles(1,euler);
        ahrs.get_NavEKF2().getVelNED(1,velNED);
        ahrs.get_NavEKF2().getPosNE(1,posNE);
        ahrs.get_NavEKF2().getPosD(1,posD);
        ahrs.get_NavEKF2().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF2().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF6_MSG),
            time_us : time_us,
            roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
            pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
            yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
            velN    : (float)(velNED.x), // velocity North (m/s)
            velE    : (float)(velNED.y), // velocity East (m/s)
            velD    : (float)(velNED.z), // velocity Down (m/s)
            posD_dot : (float)(posDownDeriv), // first derivative of down position
            posN    : (float)(posNE.x), // metres North
            posE    : (float)(posNE.y), // metres East
            posD    : (float)(posD), // metres Down
            gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
            gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
            gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
            originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
        };
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF2().getAccelZBias(1,azbias);
        ahrs.get_NavEKF2().getWind(1,wind);
        ahrs.get_NavEKF2().getMagNED(1,magNED);
        ahrs.get_NavEKF2().getMagXYZ(1,magXYZ);
        ahrs.get_NavEKF2().getGyroScaleErrorPercentage(1,gyroScaleFactor);
        magIndex = ahrs.get_NavEKF2().getActiveMag(1);
        struct log_NKF2 pkt7 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF7_MSG),
            time_us : time_us,
            AZbias  : (int8_t)(100*azbias),
            scaleX  : (int16_t)(100*gyroScaleFactor.x),
            scaleY  : (int16_t)(100*gyroScaleFactor.y),
            scaleZ  : (int16_t)(100*gyroScaleFactor.z),
            windN   : (int16_t)(100*wind.x),
            windE   : (int16_t)(100*wind.y),
            magN    : (int16_t)(magNED.x),
            magE    : (int16_t)(magNED.y),
            magD    : (int16_t)(magNED.z),
            magX    : (int16_t)(magXYZ.x),
            magY    : (int16_t)(magXYZ.y),
            magZ    : (int16_t)(magXYZ.z),
            index   : (uint8_t)(magIndex)
        };
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF2().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF8_MSG),
            time_us : time_us,
            innovVN : (int16_t)(100*velInnov.x),
            innovVE : (int16_t)(100*velInnov.y),
            innovVD : (int16_t)(100*velInnov.z),
            innovPN : (int16_t)(100*posInnov.x),
            innovPE : (int16_t)(100*posInnov.y),
            innovPD : (int16_t)(100*posInnov.z),
            innovMX : (int16_t)(magInnov.x),
            innovMY : (int16_t)(magInnov.y),
            innovMZ : (int16_t)(magInnov.z),
            innovYaw : (int16_t)(100*degrees(yawInnov)),
            innovVT : (int16_t)(100*tasInnov)
        };
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF2().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF2().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF2().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF2().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF2().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF2().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF9_MSG),
            time_us : time_us,
            sqrtvarV : (int16_t)(100*velVar),
            sqrtvarP : (int16_t)(100*posVar),
            sqrtvarH : (int16_t)(100*hgtVar),
            sqrtvarM : (int16_t)(100*tempVar),
            sqrtvarVT : (int16_t)(100*tasVar),
            tiltErr : (float)tiltError,
            offsetNorth : (int8_t)(offset.x),
            offsetEast : (int8_t)(offset.y),
            faults : (uint16_t)(faultStatus),
            timeouts : (uint8_t)(timeoutStatus),
            solution : (uint16_t)(solutionStatus.value),
            gps : (uint16_t)(gpsStatus.value),
            primary : (int8_t)primaryIndex
        };
        WriteBlock(&pkt9, sizeof(pkt9));

        ahrs.get_NavEKF2().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {
            LOG_PACKET_HEADER_INIT(LOG_NKQ2_MSG),
            time_us : time_us,
            q1 : quat.q1,
            q2 : quat.q2,
            q3 : quat.q3,
            q4 : quat.q4
        };
        WriteBlock(&pktq2, sizeof(pktq2));
    }

    // write range beacon fusion debug packet if the range value is non-zero
    if (ahrs.get_beacon() != nullptr) {
        uint8_t ID;
        float rng;
        float innovVar;
        float innov;
        float testRatio;
        Vector3f beaconPosNED;
        float bcnPosOffsetHigh;
        float bcnPosOffsetLow;
        if (ahrs.get_NavEKF2().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow)) {
            if (rng > 0.0f) {
                struct log_RngBcnDebug pkt10 = {
                    LOG_PACKET_HEADER_INIT(LOG_NKF10_MSG),
                    time_us : time_us,
                    ID : (uint8_t)ID,
                    rng : (int16_t)(100*rng),
                    innov : (int16_t)(100*innov),
                    sqrtInnovVar : (uint16_t)(100*safe_sqrt(innovVar)),
                    testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                    beaconPosN : (int16_t)(100*beaconPosNED.x),
                    beaconPosE : (int16_t)(100*beaconPosNED.y),
                    beaconPosD : (int16_t)(100*beaconPosNED.z),
                    offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                    offsetLow : (int16_t)(100*bcnPosOffsetLow),
                    posN : 0,
                    posE : 0,
                    posD : 0
                };
                WriteBlock(&pkt10, sizeof(pkt10));
            }
        }
    }

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF2().activeCores(); i++) {
            ahrs.get_NavEKF2().getTimingStatistics(i, timing);
            Log_Write_EKF_Timing(i==0?"NKT1":"NKT2", time_us, timing);
        }
    }
}


void DataFlash_Class::Log_Write_EKF3(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled)
{
    uint64_t time_us = AP_HAL::micros64();
	// Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f dAngBias;
    Vector3f dVelBias;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF3().getEulerAngles(0,euler);
    ahrs.get_NavEKF3().getVelNED(0,velNED);
    ahrs.get_NavEKF3().getPosNE(0,posNE);
    ahrs.get_NavEKF3().getPosD(0,posD);
    ahrs.get_NavEKF3().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF3().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_XKF1_MSG),
        time_us : time_us,
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    uint8_t magIndex = ahrs.get_NavEKF3().getActiveMag(0);
    ahrs.get_NavEKF3().getAccelBias(0,accelBias);
    ahrs.get_NavEKF3().getWind(0,wind);
    ahrs.get_NavEKF3().getMagNED(0,magNED);
    ahrs.get_NavEKF3().getMagXYZ(0,magXYZ);
    struct log_NKF2a pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF2_MSG),
        time_us : time_us,
        accBiasX  : (int16_t)(100*accelBias.x),
        accBiasY  : (int16_t)(100*accelBias.y),
        accBiasZ  : (int16_t)(100*accelBias.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        index   : (uint8_t)(magIndex)
    };
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF3().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF3_MSG),
        time_us : time_us,
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov)
    };
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF3().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF3().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF3().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF3().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF3().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF3().getTiltError(0,tiltError);
    uint8_t primaryIndex = ahrs.get_NavEKF3().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF4_MSG),
        time_us : time_us,
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint16_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF3().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF3().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF5_MSG),
        time_us : time_us,
        normInnov : (uint8_t)(MIN(100*normInnov,255)),
        FIX : (int16_t)(1000*flowInnovX),
        FIY : (int16_t)(1000*flowInnovY),
        AFI : (int16_t)(1000*auxFlowInnov),
        HAGL : (int16_t)(100*HAGL),
        offset : (int16_t)(100*gndOffset),
        RI : (int16_t)(100*rngInnov),
        meaRng : (uint16_t)(100*range),
        errHAGL : (uint16_t)(100*gndOffsetErr),
        angErr : (float)predictorErrors.x,
        velErr : (float)predictorErrors.y,
        posErr : (float)predictorErrors.z
     };
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF3().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {
        LOG_PACKET_HEADER_INIT(LOG_XKQ1_MSG),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    WriteBlock(&pktq1, sizeof(pktq1));
    
    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF3().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF3().getEulerAngles(1,euler);
        ahrs.get_NavEKF3().getVelNED(1,velNED);
        ahrs.get_NavEKF3().getPosNE(1,posNE);
        ahrs.get_NavEKF3().getPosD(1,posD);
        ahrs.get_NavEKF3().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF3().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF6_MSG),
            time_us : time_us,
            roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
            pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
            yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
            velN    : (float)(velNED.x), // velocity North (m/s)
            velE    : (float)(velNED.y), // velocity East (m/s)
            velD    : (float)(velNED.z), // velocity Down (m/s)
            posD_dot : (float)(posDownDeriv), // first derivative of down position
            posN    : (float)(posNE.x), // metres North
            posE    : (float)(posNE.y), // metres East
            posD    : (float)(posD), // metres Down
            gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
            gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
            gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
            originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
        };
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF3().getAccelBias(1,accelBias);
        ahrs.get_NavEKF3().getWind(1,wind);
        ahrs.get_NavEKF3().getMagNED(1,magNED);
        ahrs.get_NavEKF3().getMagXYZ(1,magXYZ);
        magIndex = ahrs.get_NavEKF3().getActiveMag(1);
        struct log_NKF2a pkt7 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF7_MSG),
            time_us : time_us,
            accBiasX  : (int16_t)(100*accelBias.x),
            accBiasY  : (int16_t)(100*accelBias.y),
            accBiasZ  : (int16_t)(100*accelBias.z),
            windN   : (int16_t)(100*wind.x),
            windE   : (int16_t)(100*wind.y),
            magN    : (int16_t)(magNED.x),
            magE    : (int16_t)(magNED.y),
            magD    : (int16_t)(magNED.z),
            magX    : (int16_t)(magXYZ.x),
            magY    : (int16_t)(magXYZ.y),
            magZ    : (int16_t)(magXYZ.z),
            index   : (uint8_t)(magIndex)
        };
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF3().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF8_MSG),
            time_us : time_us,
            innovVN : (int16_t)(100*velInnov.x),
            innovVE : (int16_t)(100*velInnov.y),
            innovVD : (int16_t)(100*velInnov.z),
            innovPN : (int16_t)(100*posInnov.x),
            innovPE : (int16_t)(100*posInnov.y),
            innovPD : (int16_t)(100*posInnov.z),
            innovMX : (int16_t)(magInnov.x),
            innovMY : (int16_t)(magInnov.y),
            innovMZ : (int16_t)(magInnov.z),
            innovYaw : (int16_t)(100*degrees(yawInnov)),
            innovVT : (int16_t)(100*tasInnov)
        };
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF3().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF3().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF3().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF3().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF3().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF3().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF9_MSG),
            time_us : time_us,
            sqrtvarV : (int16_t)(100*velVar),
            sqrtvarP : (int16_t)(100*posVar),
            sqrtvarH : (int16_t)(100*hgtVar),
            sqrtvarM : (int16_t)(100*tempVar),
            sqrtvarVT : (int16_t)(100*tasVar),
            tiltErr : (float)tiltError,
            offsetNorth : (int8_t)(offset.x),
            offsetEast : (int8_t)(offset.y),
            faults : (uint16_t)(faultStatus),
            timeouts : (uint8_t)(timeoutStatus),
            solution : (uint16_t)(solutionStatus.value),
            gps : (uint16_t)(gpsStatus.value),
            primary : (int8_t)primaryIndex
        };
        WriteBlock(&pkt9, sizeof(pkt9));

        // log quaternion
        ahrs.get_NavEKF3().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {
            LOG_PACKET_HEADER_INIT(LOG_XKQ2_MSG),
            time_us : time_us,
            q1 : quat.q1,
            q2 : quat.q2,
            q3 : quat.q3,
            q4 : quat.q4
        };
        WriteBlock(&pktq2, sizeof(pktq2));        
    }
    // write range beacon fusion debug packet if the range value is non-zero
    uint8_t ID;
    float rng;
    float innovVar;
    float innov;
    float testRatio;
    Vector3f beaconPosNED;
    float bcnPosOffsetHigh;
    float bcnPosOffsetLow;
    Vector3f posNED;
     if (ahrs.get_NavEKF3().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow, posNED)) {
        if (rng > 0.0f) {
            struct log_RngBcnDebug pkt10 = {
                LOG_PACKET_HEADER_INIT(LOG_XKF10_MSG),
                time_us : time_us,
                ID : (uint8_t)ID,
                rng : (int16_t)(100*rng),
                innov : (int16_t)(100*innov),
                sqrtInnovVar : (uint16_t)(100*sqrtf(innovVar)),
                testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                beaconPosN : (int16_t)(100*beaconPosNED.x),
                beaconPosE : (int16_t)(100*beaconPosNED.y),
                beaconPosD : (int16_t)(100*beaconPosNED.z),
                offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                offsetLow : (int16_t)(100*bcnPosOffsetLow),
                posN : (int16_t)(100*posNED.x),
                posE : (int16_t)(100*posNED.y),
                posD : (int16_t)(100*posNED.z)

             };
            WriteBlock(&pkt10, sizeof(pkt10));
        }
    }
    // write debug data for body frame odometry fusion
    Vector3f velBodyInnov,velBodyInnovVar;
    static uint32_t lastUpdateTime_ms = 0;
    uint32_t updateTime_ms = ahrs.get_NavEKF3().getBodyFrameOdomDebug(-1, velBodyInnov, velBodyInnovVar);
    if (updateTime_ms > lastUpdateTime_ms) {
        struct log_ekfBodyOdomDebug pkt11 = {
            LOG_PACKET_HEADER_INIT(LOG_XKFD_MSG),
            time_us : time_us,
            velInnovX : velBodyInnov.x,
            velInnovY : velBodyInnov.y,
            velInnovZ : velBodyInnov.z,
            velInnovVarX : velBodyInnovVar.x,
            velInnovVarY : velBodyInnovVar.y,
            velInnovVarZ : velBodyInnovVar.z
         };
        WriteBlock(&pkt11, sizeof(pkt11));
        updateTime_ms = lastUpdateTime_ms;
    }

    // log state variances every 0.49s
    static uint32_t lastEkfStateVarLogTime_ms = 0;
    if (AP_HAL::millis() - lastEkfStateVarLogTime_ms > 490) {
        lastEkfStateVarLogTime_ms = AP_HAL::millis();
        float stateVar[24];
        ahrs.get_NavEKF3().getStateVariances(-1, stateVar);
        struct log_ekfStateVar pktv1 = {
            LOG_PACKET_HEADER_INIT(LOG_XKV1_MSG),
            time_us : time_us,
            v00 : stateVar[0],
            v01 : stateVar[1],
            v02 : stateVar[2],
            v03 : stateVar[3],
            v04 : stateVar[4],
            v05 : stateVar[5],
            v06 : stateVar[6],
            v07 : stateVar[7],
            v08 : stateVar[8],
            v09 : stateVar[9],
            v10 : stateVar[10],
            v11 : stateVar[11]
        };
        WriteBlock(&pktv1, sizeof(pktv1));
        struct log_ekfStateVar pktv2 = {
            LOG_PACKET_HEADER_INIT(LOG_XKV2_MSG),
            time_us : time_us,
            v00 : stateVar[12],
            v01 : stateVar[13],
            v02 : stateVar[14],
            v03 : stateVar[15],
            v04 : stateVar[16],
            v05 : stateVar[17],
            v06 : stateVar[18],
            v07 : stateVar[19],
            v08 : stateVar[20],
            v09 : stateVar[21],
            v10 : stateVar[22],
            v11 : stateVar[23]
        };
        WriteBlock(&pktv2, sizeof(pktv2));
    }


    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF3().activeCores(); i++) {
            ahrs.get_NavEKF3().getTimingStatistics(i, timing);
            Log_Write_EKF_Timing(i==0?"XKT1":"XKT2", time_us, timing);
        }
    }
}
#endif

// Write a command processing packet
bool DataFlash_Backend::Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        time_us         : AP_HAL::micros64(),
        command_total   : (uint16_t)cmd_total,
        sequence        : (uint16_t)mav_cmd.seq,
        command         : (uint16_t)mav_cmd.command,
        param1          : (float)mav_cmd.param1,
        param2          : (float)mav_cmd.param2,
        param3          : (float)mav_cmd.param3,
        param4          : (float)mav_cmd.param4,
        latitude        : (float)mav_cmd.x,
        longitude       : (float)mav_cmd.y,
        altitude        : (float)mav_cmd.z
    };
    return WriteBlock(&pkt, sizeof(pkt));
}

void DataFlash_Class::Log_Write_Radio(const mavlink_radio_t &packet)
{
    struct log_Radio pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RADIO_MSG),
        time_us      : AP_HAL::micros64(),
        rssi         : packet.rssi,
        remrssi      : packet.remrssi,
        txbuf        : packet.txbuf,
        noise        : packet.noise,
        remnoise     : packet.remnoise,
        rxerrors     : packet.rxerrors,
        fixed        : packet.fixed
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void DataFlash_Class::Log_Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc)
{
    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.flags.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : AP_HAL::micros64(),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    WriteCriticalBlock(&pkt, sizeof(pkt));
}


// Write a Pos packet
void DataFlash_Class::Pos_Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc)
{
    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.flags.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : AP_HAL::micros64(),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    WritePosData(&pkt, sizeof(pkt));
}

// Write a Camera packet
void DataFlash_Class::Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc)
{		
    Log_Write_CameraInfo(LOG_CAMERA_MSG, ahrs, gps, current_loc);
}

// Write a Trigger packet
void DataFlash_Class::Log_Write_Trigger(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc)
{
    Log_Write_CameraInfo(LOG_TRIGGER_MSG, ahrs, gps, current_loc);
}

// Write an attitude packet
void DataFlash_Class::Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)targets.z,
        yaw             : (uint16_t)ahrs.yaw_sensor,
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void DataFlash_Class::Log_Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)targets.z,
        yaw             : (uint16_t)ahrs.yaw_sensor,
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an Current data packet
void DataFlash_Class::Log_Write_Current(const AP_BattMonitor &battery)
{
    if (battery.num_instances() >= 1) {
        float temp;
        bool has_temp = battery.get_temperature(temp, 0);
        struct log_Current pkt = {
            LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
            time_us             : AP_HAL::micros64(),
            voltage             : battery.voltage(0),
            copter_voltage      : battery.copter_voltage(0),
            steer_voltage       : battery.steer_voltage(0),
            voltage_resting     : battery.voltage_resting_estimate(0),
            current_amps        : battery.current_amps(0),
            current_total       : battery.current_total_mah(0),
            temperature         : (int16_t)(has_temp ? (temp * 100) : 0),
            resistance          : battery.get_resistance(0)
        };
        WriteBlock(&pkt, sizeof(pkt));

        // individual cell voltages
        if (battery.has_cell_voltages(0)) {
            const AP_BattMonitor::cells &cells = battery.get_cell_voltages(0);
            struct log_Current_Cells cell_pkt = {
                LOG_PACKET_HEADER_INIT(LOG_CURRENT_CELLS_MSG),
                time_us             : AP_HAL::micros64(),
                voltage             : battery.voltage(0)
            };
            for (uint8_t i = 0; i < ARRAY_SIZE(cells.cells); i++) {
                cell_pkt.cell_voltages[i] = cells.cells[i] + 1;
            }
            WriteBlock(&cell_pkt, sizeof(cell_pkt));

            // check battery structure can hold all cells
            static_assert(ARRAY_SIZE(cells.cells) == (sizeof(cell_pkt.cell_voltages) / sizeof(cell_pkt.cell_voltages[0])),
                          "Battery cell number doesn't match in library and log structure");
        }
    }

    if (battery.num_instances() >= 2) {
        float temp;
        bool has_temp = battery.get_temperature(temp, 1);
        struct log_Current pkt = {
            LOG_PACKET_HEADER_INIT(LOG_CURRENT2_MSG),
            time_us             : AP_HAL::micros64(),
            voltage             : battery.voltage(1),
            copter_voltage      : battery.copter_voltage(1),
            steer_voltage       : battery.steer_voltage(1),
            voltage_resting     : battery.voltage_resting_estimate(1),
            current_amps        : battery.current_amps(1),
            current_total       : battery.current_total_mah(1),
            temperature         : (int16_t)(has_temp ? (temp * 100) : 0),
            resistance          : battery.get_resistance(1)
        };
        WriteBlock(&pkt, sizeof(pkt));

        // individual cell voltages
        if (battery.has_cell_voltages(1)) {
            const AP_BattMonitor::cells &cells = battery.get_cell_voltages(1);
            struct log_Current_Cells cell_pkt = {
                LOG_PACKET_HEADER_INIT(LOG_CURRENT_CELLS_MSG),
                time_us             : AP_HAL::micros64(),
                voltage             : battery.voltage(1)
            };
            for (uint8_t i = 0; i < ARRAY_SIZE(cells.cells); i++) {
                cell_pkt.cell_voltages[i] = cells.cells[i] + 1;
            }
            WriteBlock(&cell_pkt, sizeof(cell_pkt));
        }
    }
}

void DataFlash_Class::Log_Write_Raw_Data(const AP_SerialManager &manager)
{
	AP_HAL::UARTDriver *uart_raw;
	uint8_t buffer[1024] = {0};

#ifdef RAWDATA_DEBUG
    static uint16_t x = 0;
    static uint32_t cnts = 0;
    if(x++ == 100) {
        printf("raw_data_update %d bytes\n", cnts);
        cnts = 0;
        x = 0;
    }
#endif

	uart_raw = manager.find_serial(AP_SerialManager::SerialProtocol_Nova_Rtcm, 0);
	if(uart_raw != nullptr) {
		uint32_t nbytes = uart_raw->available();
        if(nbytes > 0) {
            if(nbytes > sizeof(buffer)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Log_Write_Raw_Data over len %d", nbytes);
                //printf( "Log_Write_Raw_Data over len %d\n", nbytes);
                nbytes = sizeof(buffer);
            }

            nbytes = uart_raw->read_bytes(buffer, nbytes);
            WriteRawData(buffer, nbytes);
#ifdef RAWDATA_DEBUG
            cnts += nbytes;
#endif
        }
	}
}

// Write a Compass packet
void DataFlash_Class::Log_Write_Compass(const Compass &compass, uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const Vector3f &mag_field = compass.get_field(0);
    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(0);
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_us         : time_us,
        mag_x           : (int16_t)mag_field.x,
        mag_y           : (int16_t)mag_field.y,
        mag_z           : (int16_t)mag_field.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z,
        health          : (uint8_t)compass.healthy(0),
        SUS             : compass.last_update_usec(0)
    };
    WriteBlock(&pkt, sizeof(pkt));

    if (compass.get_count() > 1) {
        const Vector3f &mag_field2 = compass.get_field(1);
        const Vector3f &mag_offsets2 = compass.get_offsets(1);
        const Vector3f &mag_motor_offsets2 = compass.get_motor_offsets(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_us         : time_us,
            mag_x           : (int16_t)mag_field2.x,
            mag_y           : (int16_t)mag_field2.y,
            mag_z           : (int16_t)mag_field2.z,
            offset_x        : (int16_t)mag_offsets2.x,
            offset_y        : (int16_t)mag_offsets2.y,
            offset_z        : (int16_t)mag_offsets2.z,
            motor_offset_x  : (int16_t)mag_motor_offsets2.x,
            motor_offset_y  : (int16_t)mag_motor_offsets2.y,
            motor_offset_z  : (int16_t)mag_motor_offsets2.z,
            health          : (uint8_t)compass.healthy(1),
            SUS             : compass.last_update_usec(1)
        };
        WriteBlock(&pkt2, sizeof(pkt2));
    }

    if (compass.get_count() > 2) {
        const Vector3f &mag_field3 = compass.get_field(2);
        const Vector3f &mag_offsets3 = compass.get_offsets(2);
        const Vector3f &mag_motor_offsets3 = compass.get_motor_offsets(2);
        struct log_Compass pkt3 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS3_MSG),
            time_us         : time_us,
            mag_x           : (int16_t)mag_field3.x,
            mag_y           : (int16_t)mag_field3.y,
            mag_z           : (int16_t)mag_field3.z,
            offset_x        : (int16_t)mag_offsets3.x,
            offset_y        : (int16_t)mag_offsets3.y,
            offset_z        : (int16_t)mag_offsets3.z,
            motor_offset_x  : (int16_t)mag_motor_offsets3.x,
            motor_offset_y  : (int16_t)mag_motor_offsets3.y,
            motor_offset_z  : (int16_t)mag_motor_offsets3.z,
            health          : (uint8_t)compass.healthy(2),
            SUS             : compass.last_update_usec(2)
        };
        WriteBlock(&pkt3, sizeof(pkt3));
    }
}

// Write a mode packet.
bool DataFlash_Backend::Log_Write_Mode(uint8_t mode, uint8_t reason)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_us  : AP_HAL::micros64(),
        mode     : mode,
        mode_num : mode,
        mode_reason : reason
    };
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write ESC status messages
void DataFlash_Class::Log_Write_ESC(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    static int _esc_status_sub = -1;
    struct esc_status_s esc_status;

    if (_esc_status_sub == -1) {
        // subscribe to ORB topic on first call
        _esc_status_sub = orb_subscribe(ORB_ID(esc_status));
    }

    // check for new ESC status data
    bool esc_updated = false;
    orb_check(_esc_status_sub, &esc_updated);
    if (esc_updated && (OK == orb_copy(ORB_ID(esc_status), _esc_status_sub, &esc_status))) {
        if (esc_status.esc_count > 8) {
            esc_status.esc_count = 8;
        }
        uint64_t time_us = AP_HAL::micros64();
        for (uint8_t i = 0; i < esc_status.esc_count; i++) {
            // skip logging ESCs with a esc_address of zero, and this
            // are probably not populated. The Pixhawk itself should
            // be address zero
            if (esc_status.esc[i].esc_address != 0) {
                struct log_Esc pkt = {
                    LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ESC1_MSG + i)),
                    time_us     : time_us,
                    rpm         : (int16_t)(esc_status.esc[i].esc_rpm/10),
                    voltage     : (int16_t)(esc_status.esc[i].esc_voltage*100.0f + .5f),
                    current     : (int16_t)(esc_status.esc[i].esc_current*100.0f + .5f),
                    temperature : (int16_t)(esc_status.esc[i].esc_temperature*100.0f + .5f)
                };

                WriteBlock(&pkt, sizeof(pkt));
            }
        }
    }
#endif // CONFIG_HAL_BOARD
}

// Write a AIRSPEED packet
void DataFlash_Class::Log_Write_Airspeed(AP_Airspeed &airspeed)
{
    uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!airspeed.enabled(i)) {
            continue;
        }
        float temperature;
        if (!airspeed.get_temperature(i, temperature)) {
            temperature = 0;
        }
        struct log_AIRSPEED pkt = {
            LOG_PACKET_HEADER_INIT(i==0?LOG_ARSP_MSG:LOG_ASP2_MSG),
            time_us       : now,
            airspeed      : airspeed.get_raw_airspeed(i),
            diffpressure  : airspeed.get_differential_pressure(i),
            temperature   : (int16_t)(temperature * 100.0f),
            rawpressure   : airspeed.get_corrected_pressure(i),
            offset        : airspeed.get_offset(i),
            use           : airspeed.use(i),
            healthy       : airspeed.healthy(i),
            primary       : airspeed.get_primary()
        };
        WriteBlock(&pkt, sizeof(pkt));
    }
}

// Write a Yaw PID packet
void DataFlash_Class::Log_Write_PID(uint8_t msg_type, const PID_Info &info)
{
    struct log_PID pkt = {
        LOG_PACKET_HEADER_INIT(msg_type),
        time_us         : AP_HAL::micros64(),
        desired         : info.desired,
        P               : info.P,
        I               : info.I,
        D               : info.D,
        FF              : info.FF,
        AFF             : info.AFF
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void DataFlash_Class::Log_Write_Origin(uint8_t origin_type, const Location &loc)
{
    uint64_t time_us = AP_HAL::micros64();
    struct log_ORGN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ORGN_MSG),
        time_us     : time_us,
        origin_type : origin_type,
        latitude    : loc.lat,
        longitude   : loc.lng,
        altitude    : loc.alt
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void DataFlash_Class::Log_Write_RPM(const AP_RPM &rpm_sensor)
{
    struct log_RPM pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
        time_us     : AP_HAL::micros64(),
        rpm1        : rpm_sensor.get_rpm(0),
        rpm2        : rpm_sensor.get_rpm(1)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void DataFlash_Class::Log_Write_Rate(const AP_AHRS &ahrs,
                                     const AP_Motors &motors,
                                     const AC_AttitudeControl &attitude_control,
                                     const AC_PosControl &pos_control)
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target();
    struct log_Rate pkt_rate = {
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : degrees(rate_targets.x),
        roll            : degrees(ahrs.get_gyro().x),
        roll_out        : motors.get_roll(),
        control_pitch   : degrees(rate_targets.y),
        pitch           : degrees(ahrs.get_gyro().y),
        pitch_out       : motors.get_pitch(),
        control_yaw     : degrees(rate_targets.z),
        yaw             : degrees(ahrs.get_gyro().z),
        yaw_out         : motors.get_yaw(),
        control_accel   : (float)accel_target.z,
        accel           : (float)(-(ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f),
        accel_out       : motors.get_throttle()
    };
    WriteBlock(&pkt_rate, sizeof(pkt_rate));
}

// Write rally points
void DataFlash_Class::Log_Write_Rally(const AP_Rally &rally)
{
    RallyLocation rally_point;
    for (uint8_t i=0; i<rally.get_rally_total(); i++) {
        if (rally.get_rally_point_with_index(i, rally_point)) {
            struct log_Rally pkt_rally = {
                LOG_PACKET_HEADER_INIT(LOG_RALLY_MSG),
                time_us         : AP_HAL::micros64(),
                total           : rally.get_rally_total(),
                sequence        : i,
                latitude        : rally_point.lat,
                longitude       : rally_point.lng,
                altitude        : rally_point.alt
            };
            WriteBlock(&pkt_rally, sizeof(pkt_rally));
        }
    }
}

// Write visual odometry sensor data
void DataFlash_Class::Log_Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence)
{
    struct log_VisualOdom pkt_visualodom = {
        LOG_PACKET_HEADER_INIT(LOG_VISUALODOM_MSG),
        time_us             : AP_HAL::micros64(),
        time_delta          : time_delta,
        angle_delta_x       : angle_delta.x,
        angle_delta_y       : angle_delta.y,
        angle_delta_z       : angle_delta.z,
        position_delta_x    : position_delta.x,
        position_delta_y    : position_delta.y,
        position_delta_z    : position_delta.z,
        confidence          : confidence
    };
    WriteBlock(&pkt_visualodom, sizeof(log_VisualOdom));
}

// Write AOA and SSA
void DataFlash_Class::Log_Write_AOA_SSA(AP_AHRS &ahrs)
{
    struct log_AOA_SSA aoa_ssa = {
        LOG_PACKET_HEADER_INIT(LOG_AOA_SSA_MSG),
        time_us         : AP_HAL::micros64(),
        AOA             : ahrs.getAOA(),
        SSA             : ahrs.getSSA()
    };

    WriteBlock(&aoa_ssa, sizeof(aoa_ssa));
}

// Write beacon sensor (position) data
void DataFlash_Class::Log_Write_Beacon(AP_Beacon &beacon)
{
    // position
    Vector3f pos;
    float accuracy = 0.0f;
    beacon.get_vehicle_position_ned(pos, accuracy);

    struct log_Beacon pkt_beacon = {
       LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
       time_us         : AP_HAL::micros64(),
       health          : (uint8_t)beacon.healthy(),
       count           : (uint8_t)beacon.count(),
       dist0           : beacon.beacon_distance(0),
       dist1           : beacon.beacon_distance(1),
       dist2           : beacon.beacon_distance(2),
       dist3           : beacon.beacon_distance(3),
       posx            : pos.x,
       posy            : pos.y,
       posz            : pos.z
    };
    WriteBlock(&pkt_beacon, sizeof(pkt_beacon));
}

// Write proximity sensor distances
void DataFlash_Class::Log_Write_Proximity(AP_Proximity &proximity)
{
    // exit immediately if not enabled
    if (proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    AP_Proximity::Proximity_Distance_Array dist_array {};
    proximity.get_horizontal_distances(dist_array);

    float dist_up;
    if (!proximity.get_upward_distance(dist_up)) {
        dist_up = 0.0f;
    }

    float close_ang = 0.0f, close_dist = 0.0f;
    proximity.get_closest_object(close_ang, close_dist);

    struct log_Proximity pkt_proximity = {
            LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_MSG),
            time_us         : AP_HAL::micros64(),
            health          : (uint8_t)proximity.get_status(),
            dist0           : dist_array.distance[0],
            dist45          : dist_array.distance[1],
            dist90          : dist_array.distance[2],
            dist135         : dist_array.distance[3],
            dist180         : dist_array.distance[4],
            dist225         : dist_array.distance[5],
            dist270         : dist_array.distance[6],
            dist315         : dist_array.distance[7],
            distup          : dist_up,
            closest_angle   : close_ang,
            closest_dist    : close_dist
    };
    WriteBlock(&pkt_proximity, sizeof(pkt_proximity));
}
