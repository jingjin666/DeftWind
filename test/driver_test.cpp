/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file driver_test.cpp
 * hello_world_cxx application example for DP deftwind
 *
 * @author Example User <mail@example.com>
 */

#include <cstdio>
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/init.h>
#include <nuttx/sched.h>

#include <apps/platform/cxxinitialize.h>

#include <dp_task.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if 1
class Plane_test : public AP_HAL::HAL::Callbacks {
public:
    Plane_test(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;
    
private:
    // main loop scheduler
    AP_Scheduler ap_scheduler;
    
public:
    void one_second_loop();
};

Plane_test:: Plane_test(void){

}

Plane_test plane_test;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Plane_test, &plane_test, func, rate_hz, max_time_micros)

static const AP_Scheduler::Task scheduler_test_tasks[]= {
                           // Units:   Hz      us
    SCHED_TASK(one_second_loop,         50,    400),
};

void Plane_test::one_second_loop() {

#define UART_TEST 1
#ifdef UART_TEST
    int rx = 0;
    const char *send = "123456789";
    char recv[1024] = {0};

    AP_HAL::UARTDriver* uart = hal.uartB;

    if(uart != nullptr) {
        uart->write((uint8_t *)send, strlen(send));
        return;
        rx = uart->available();
        
        if((rx > 0) && (rx <= sizeof(recv))) {
            for(int i = 0; i < rx; i++) {
                recv[i] = uart->read();
                //printf("%c", recv[i]);
            }
            //printf("rx is %d\n", rx);
            //printf("%s\n", recv);
            uart->write((uint8_t *)&recv[0], rx);
        } else {
            //printf("rx is %d\n", rx);
        }
    }
#endif

#ifdef GPIO_TEST
    int value = hal.gpio->imu_data_ready();
    if(value == 0)
        printf("value is %d\n", value);
#endif

    //printf("loop---------------\n");
}


void Plane_test::setup() 
{
    hal.storage->init();
    
    // initialise the main loop scheduler
    ap_scheduler.init(&scheduler_test_tasks[0], ARRAY_SIZE(scheduler_test_tasks));
}

void Plane_test::loop()
{
    uint32_t loop_us = 1000000UL / ap_scheduler.get_loop_rate_hz();

    hal.scheduler->delay_microseconds(loop_us);

    // tell the scheduler one tick has passed
    ap_scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    ap_scheduler.run(loop_us);
}

AP_HAL_MAIN_CALLBACKS(&plane_test);
#else
//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: DeftWind_main
 ****************************************************************************/

extern "C"
{
static bool thread_should_exit = false;     /**< deftwind exit flag */
static bool thread_running = false;     /**< deftwind status flag */
static int deftwind_task;                /**< Handle of deftwind task / thread */

 __EXPORT int DeftWind_main(int argc, char *argv[]);

 int deftwind(int argc, char *argv[])
 {
 
     printf("[deftwind] starting\n");
 
     thread_running = true;
 
     while (!thread_should_exit) {
         printf("deftwind!\n");
         sleep(1);
     }
 
     printf("[deftwind] exiting.\n");
 
     thread_running = false;
 
     return 0;
 }

 int DeftWind_main(int argc, char *argv[])
 {
	// If C++ initialization for static constructors is supported, then do
	// that first

	up_cxxinitialize();

    deftwind_task = dp_task_create("deftwind",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     2000,
                     deftwind,
                     (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
    return 0;
}

}
#endif
