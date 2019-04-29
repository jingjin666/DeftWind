#pragma once

#include "AP_HAL_UAVRS.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include "perf_counter.h"
#include <AP_HAL/AP_HAL.h>




#define UAVRS_SCHEDULER_MAX_TIMER_PROCS 8

#define UAVRS_MAIN_PRIORITY_BOOST 241
#define UAVRS_MAIN_PRIORITY       180
#define UAVRS_TIMER_PRIORITY      181
#define UAVRS_SPI_PRIORITY        242
#define UAVRS_CAN_PRIORITY        179
#define UAVRS_I2C_PRIORITY        178
#define UAVRS_UART_PRIORITY        60
#define UAVRS_STORAGE_PRIORITY     59
#define UAVRS_IO_PRIORITY          58
#define UAVRS_SHELL_PRIORITY       57
#define UAVRS_OVERTIME_PRIORITY    10
#define UAVRS_STARTUP_PRIORITY     10

#define UAVRS_MAIN_PRIORITY_BOOST_USEC 150

#define UAVRS_MAIN_THREAD_STACK_SIZE 8192


class UAVRS::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us);

    void     system_initialized();

    void     reboot(bool hold_in_bootloader);
    
    bool     in_main_thread() const;

private:
    bool _initialized;
    volatile bool _hal_initialized;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    AP_HAL::Proc _failsafe;

    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[UAVRS_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[UAVRS_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pid_t _main_task_pid;
    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _storage_thread_ctx;
    pthread_t _uart_thread_ctx;

    static void *_timer_thread(void *arg);
    static void *_io_thread(void *arg);
    static void *_storage_thread(void *arg);
    static void *_uart_thread(void *arg);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);

    void delay_microseconds_semaphore(uint16_t usec);

    perf_counter_t  _perf_timers;
    perf_counter_t  _perf_io_timers;
    perf_counter_t  _perf_storage_timer;
    perf_counter_t  _perf_delay;
};
