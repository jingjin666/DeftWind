
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nshlib/nshlib.h>
#include <fcntl.h>
#include "UARTDriver.h"
#include <uORB/uORB.h>
#include <uORB/topics/safety.h>
#include <systemlib/board_serial.h>
#include <drivers/drv_gpio.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace UAVRS;

extern bool _uavrs_thread_should_exit;

/*
  constructor
 */
UAVRSUtil::UAVRSUtil(void) : Util()
{
    _safety_handle = orb_subscribe(ORB_ID(safety));
}

/*
  start an instance of nsh
 */
bool UAVRSUtil::run_debug_shell(AP_HAL::BetterStream *stream)
{
    return true;
}

/*
  return state of safety switch
 */
enum UAVRSUtil::safety_state UAVRSUtil::safety_switch_state(void)
{
#if !HAL_HAVE_SAFETY_SWITCH
    return AP_HAL::Util::SAFETY_NONE;
#endif

    if (_safety_handle == -1) {
        _safety_handle = orb_subscribe(ORB_ID(safety));
    }
    if (_safety_handle == -1) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    struct safety_s safety;
    if (orb_copy(ORB_ID(safety), _safety_handle, &safety) != OK) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    if (!safety.safety_switch_available) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    if (safety.safety_off) {
        return AP_HAL::Util::SAFETY_ARMED;
    }
    return AP_HAL::Util::SAFETY_DISARMED;
}

void UAVRSUtil::set_system_clock(uint64_t time_utc_usec)
{
    timespec ts;
    ts.tv_sec = time_utc_usec/1000000ULL;
    ts.tv_nsec = (time_utc_usec % 1000000ULL) * 1000ULL;
    clock_settime(CLOCK_REALTIME, &ts);    
}


/*
  display UAVRS system identifer - board type and serial number
 */
bool UAVRSUtil::get_system_id(char buf[40])
{
    uint8_t serialid[12];
    memset(serialid, 0, sizeof(serialid));
    get_board_serial(serialid);
#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
    const char *board_type = "UAVRSv1";
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    const char *board_type = "UAVRSv2";
#else
    const char *board_type = "UAVRSv?";
#endif
    // this format is chosen to match the human_readable_serial()
    // function in auth.c
    snprintf(buf, 40, "%s %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             board_type,
             (unsigned)serialid[0], (unsigned)serialid[1], (unsigned)serialid[2], (unsigned)serialid[3], 
             (unsigned)serialid[4], (unsigned)serialid[5], (unsigned)serialid[6], (unsigned)serialid[7], 
             (unsigned)serialid[8], (unsigned)serialid[9], (unsigned)serialid[10],(unsigned)serialid[11]); 
    return true;
}

/**
   how much free memory do we have in bytes.
*/
uint32_t UAVRSUtil::available_memory(void) 
{
    //struct mallinfo info = mallinfo();
    //return info.fordblks;
    return 160*1024;
}

/**
   how much total memory do we have in bytes.
*/
uint32_t UAVRSUtil::total_memory(void)
{
    //struct mallinfo info = mallinfo();
    //return info.arena;
    return 300*1024;
}

/*
  AP_HAL wrapper around UAVRS perf counters
 */
UAVRSUtil::perf_counter_t UAVRSUtil::perf_alloc(UAVRSUtil::perf_counter_type t, const char *name)
{
    ::perf_counter_type uavrs_t;
    switch (t) {
    case UAVRSUtil::PC_COUNT:
        uavrs_t = ::PC_COUNT;
        break;
    case UAVRSUtil::PC_ELAPSED:
        uavrs_t = ::PC_ELAPSED;
        break;
    case UAVRSUtil::PC_INTERVAL:
        uavrs_t = ::PC_INTERVAL;
        break;
    default:
        return nullptr;
    }
    return (perf_counter_t)::perf_alloc(uavrs_t, name);
}

void UAVRSUtil::perf_begin(perf_counter_t h)
{
    ::perf_begin((::perf_counter_t)h);
}

void UAVRSUtil::perf_end(perf_counter_t h)
{
    ::perf_end((::perf_counter_t)h);
}

void UAVRSUtil::perf_count(perf_counter_t h)
{
    ::perf_count((::perf_counter_t)h);
}

void UAVRSUtil::set_imu_temp(float current)
{
    if (!_heater.target || *_heater.target == -1) {
        return;
    }

    // average over temperatures to remove noise
    _heater.count++;
    _heater.sum += current;
    
    // update once a second
    uint32_t now = AP_HAL::millis();
    if (now - _heater.last_update_ms < 1000) {
        return;
    }
    _heater.last_update_ms = now;

    current = _heater.sum / _heater.count;
    _heater.sum = 0;
    _heater.count = 0;

    // experimentally tweaked for Pixhawk2
    const float kI = 0.3f;
    const float kP = 200.0f;
    float target = (float)(*_heater.target);

    // limit to 65 degrees to prevent damage
    target = constrain_float(target, 0, 65);
    
    float err = target - current;

    _heater.integrator += kI * err;
    _heater.integrator = constrain_float(_heater.integrator, 0, 70);

    float output = constrain_float(kP * err + _heater.integrator, 0, 100);
    
    //printf("integrator %.1f out=%.1f temp=%.2f err=%.2f\n", _heater.integrator, output, current, err);
}


void UAVRSUtil::set_imu_target_temp(int8_t *target)
{
    _heater.target = target;
}


extern "C" {
    extern void *fat_dma_alloc(size_t);
    extern void fat_dma_free(void *, size_t);
}

/*
  allocate DMA-capable memory if possible. Otherwise return normal
  memory.
*/
void *UAVRSUtil::dma_allocate(size_t size)
{
    return fat_dma_alloc(size);
}
void UAVRSUtil::dma_free(void *ptr, size_t size)
{
    fat_dma_free(ptr, size);
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
