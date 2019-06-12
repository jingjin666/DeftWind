
#include "Scheduler.h"

#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <nuttx/arch.h>
#include <pthread.h>
#include <poll.h>

#include "UARTDriver.h"
#include "Storage.h"
#include "RCOutput.h"
#include "AnalogIn.h"
#include "RCInput.h"
#include <AP_Scheduler/AP_Scheduler.h>



using namespace UAVRS;

extern const AP_HAL::HAL& hal;

extern bool _uavrs_thread_should_exit;


Scheduler::Scheduler():
    _perf_timers(perf_alloc(PC_ELAPSED, "UAVRS_timers")),
    _perf_io_timers(perf_alloc(PC_ELAPSED, "UAVRS_IO_timers")),
    _perf_storage_timer(perf_alloc(PC_ELAPSED, "UAVRS_storage_timers")),
    _perf_delay(perf_alloc(PC_ELAPSED, "UAVRS_delay"))
{}

void Scheduler::init()
{
    _main_task_pid = getpid();

    // setup the timer thread - this will call tasks at 1kHz
    pthread_attr_t thread_attr;
    struct sched_param param;

    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = UAVRS_TIMER_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_timer_thread_ctx, &thread_attr, &Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = UAVRS_UART_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_uart_thread_ctx, &thread_attr, &Scheduler::_uart_thread, this);
#if 0
    // the IO thread runs at lower priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = UAVRS_IO_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_io_thread_ctx, &thread_attr, &Scheduler::_io_thread, this);
#endif
    // the storage thread runs at just above IO priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 1024);

    param.sched_priority = UAVRS_STORAGE_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_storage_thread_ctx, &thread_attr, &Scheduler::_storage_thread, this);
}

void Scheduler::delay(uint16_t ms)
{
    if (in_timerprocess()) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
    perf_begin(_perf_delay);
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms &&
           !_uavrs_thread_should_exit) {
        delay_microseconds_semaphore(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
    perf_end(_perf_delay);
    if (_uavrs_thread_should_exit) {
        exit(1);
    }
}

/**
   delay for a specified number of microseconds using a semaphore wait
 */
void Scheduler::delay_microseconds_semaphore(uint16_t usec)
{
    sem_t wait_semaphore;
    struct hrt_call wait_call;
    sem_init(&wait_semaphore, 0, 0);
    memset(&wait_call, 0, sizeof(wait_call));
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post, &wait_semaphore);
    sem_wait(&wait_semaphore);
}


void Scheduler::delay_microseconds(uint16_t us)
{
    perf_begin(_perf_delay);
    delay_microseconds_semaphore(us);
    perf_end(_perf_delay);
}

void Scheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{
    _delay_cb = k;
    _min_delay_cb_ms = min_time_ms;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == k) {
            return;
        }
    }

    if (_num_timer_procs < UAVRS_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = k;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == k) {
            return;
        }
    }

    if (_num_io_procs < UAVRS_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = k;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

bool Scheduler::in_timerprocess() {
    return getpid() != _main_task_pid;
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    // delay to ensure the async force_saftey operation completes
    delay(500);

    exit(0);
}

bool Scheduler::in_main_thread() const
{
    return getpid() == _main_task_pid;
}



void Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    // process analog input
    ((AnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

extern bool uavrs_ran_overtime;

void *Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    uint32_t last_ran_overtime = 0;

    pthread_setname_np(pthread_self(), "uavrs_timer");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_uavrs_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // run registered timers
        perf_begin(sched->_perf_timers);
        sched->_run_timers(true);
        perf_end(sched->_perf_timers);
#if 0
        // process any pending RC output requests
        hal.rcout->timer_tick();

        // process any pending RC input requests
        ((RCInput *)hal.rcin)->_timer_tick();
#endif
        if (uavrs_ran_overtime && AP_HAL::millis() - last_ran_overtime > 2000) {
            last_ran_overtime = AP_HAL::millis();
#if 0
            printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
            hal.console->printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
#endif
        }
    }
    return nullptr;
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *Scheduler::_uart_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    pthread_setname_np(pthread_self(), "uavrs_uart");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_uavrs_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // process any pending serial bytes
        ((UARTDriver *)hal.uartA)->_timer_tick();
        ((UARTDriver *)hal.uartB)->_timer_tick();
        ((UARTDriver *)hal.uartC)->_timer_tick();
        ((UARTDriver *)hal.uartD)->_timer_tick();
        ((UARTDriver *)hal.uartE)->_timer_tick();
        ((UARTDriver *)hal.uartF)->_timer_tick();
        ((UARTDriver *)hal.uartG)->_timer_tick();
    }
    printf("_uart_thread exit\n");
    return nullptr;
}

void *Scheduler::_io_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    pthread_setname_np(pthread_self(), "uavrs_io");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_uavrs_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // run registered IO processes
        perf_begin(sched->_perf_io_timers);
        sched->_run_io();
        perf_end(sched->_perf_io_timers);
    }
    return nullptr;
}

void *Scheduler::_storage_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    pthread_setname_np(pthread_self(), "uavrs_storage");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_uavrs_thread_should_exit) {
        sched->delay_microseconds_semaphore(10000);

        // process any pending storage writes
        perf_begin(sched->_perf_storage_timer);
        ((Storage *)hal.storage)->_timer_tick();
        perf_end(sched->_perf_storage_timer);
    }
    return nullptr;
}
