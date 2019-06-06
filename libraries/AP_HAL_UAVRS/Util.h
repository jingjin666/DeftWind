#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_UAVRS_Namespace.h"
#include "Semaphores.h"

class UAVRS::UAVRSUtil : public AP_HAL::Util {
public:
    UAVRSUtil(void);
    bool run_debug_shell(AP_HAL::BetterStream *stream);

    enum safety_state safety_switch_state(void);

    /*
      set system clock in UTC microseconds
     */
    void set_system_clock(uint64_t time_utc_usec);

    /*
      get system identifier (STM32 serial number)
     */
    bool get_system_id(char buf[40]);

    uint32_t available_memory(void) override;

    /*
      return a stream for access to nsh shell
     */
    AP_HAL::Stream *get_shell_stream() { return nullptr; }
    perf_counter_t perf_alloc(perf_counter_type t, const char *name) override;
    void perf_begin(perf_counter_t ) override;
    void perf_end(perf_counter_t) override;
    void perf_count(perf_counter_t) override;
    
    // create a new semaphore
    AP_HAL::Semaphore *new_semaphore(void) override { return new UAVRS::Semaphore; }

    void set_imu_temp(float current) override;
    void set_imu_target_temp(int8_t *target) override;

    // allocate and free DMA-capable memory if possible. Otherwise return normal memory
    void *dma_allocate(size_t size) override;
    void dma_free(void *ptr, size_t size) override;
    
private:
    int _safety_handle;

    struct {
        int8_t *target;
        float integrator;
        uint16_t count;
        float sum;
        uint32_t last_update_ms;
        int fd = -1;
    } _heater;
};
