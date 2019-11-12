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

#include <drivers/drv_hrt.h>
#include <fcntl.h>
#define TEST_FILE "/fs/microsd/UAVRS/RAW_DATA/test.bin"

static void fifo_dump(uint8_t *fifo, int size, int per)
{
    for(int k = 0; k < size; k++) {
        if((k!=0) && (k%per == 0))
            printf("\n");

        printf("0x%02x, ", fifo[k]);
    }
    printf("\n");
}

void Plane_test::one_second_loop()
{
#if 0
    const char *uart_name = "/dev/ttyS1";
    char sample_test_uart[1024] = {0};
    static int test_uart_fd = -1;
    uint32_t nread = 0, nwrite = 0;
    static uint32_t rx_cnts = 0;

    static uint8_t x = 0;
    if(x++ == 50) {
        printf("one_second_loop %d bytes\n", rx_cnts);
        x = 0;
    }

    if(test_uart_fd == -1) {
        test_uart_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

        if (test_uart_fd < 0) {
            printf("ERROR opening UART %s, aborting..\n", uart_name);
            return ;
        } else {
            printf("R/W test to UART %s\n", uart_name);
        }
    } else {
        if (ioctl(test_uart_fd, FIONREAD, (unsigned long)&nread) == 0) {
            if (nread > sizeof(sample_test_uart)) {
                printf("nread is %d\n", nread);
                nread = sizeof(sample_test_uart);
            }
            if (nread > 0) {
                int ret = read(test_uart_fd, sample_test_uart, nread);
                rx_cnts += ret;
            }
        }
    }
#else
    int rx = 0;
    const char *send = "123456789";
    uint8_t recv[1024] = {0};
    static int fd = -1;
    int ret;
    static int cnts_wirte = 0;

    AP_HAL::UARTDriver* uart = hal.uartB;
    
    static uint8_t x = 0;
    if(x++ == 50) {
        printf("one_second_loop %d bytes\n", cnts_wirte);
        x = 0;
        if(rx > 0) {
            printf("rx is %d\n", rx);
        }
    }
    
#if 1
    if(cnts_wirte < 2*1024*1024) {
        if(fd == -1) {
            fd = ::open(TEST_FILE, O_WRONLY | O_CREAT | O_TRUNC);
            if(-1 == fd) {
                printf("%s open failed\n", TEST_FILE);
                return ;
            } else {
                printf("%s open succuess\n", TEST_FILE);
            }
        }
#endif
        if(uart != nullptr) {
            //uart->write((uint8_t *)send, strlen(send));
            //return;
            rx = uart->available();
            if(rx > 0) {
                if(rx >= sizeof(recv))
                    rx = sizeof(recv);
                
                #if 0
                for(int i = 0; i < rx; i++) {
                    recv[i] = uart->read();
                    //printf("%c", recv[i]);
                }
                #else
                uart->read_bytes(recv, rx);
                #endif
                
                //printf("rx is %d\n", rx);
                //printf("%s\n", recv);
                //uart->write(recv, rx);
                
                #if 1
                if(fd > 0) {
                    ret = ::write(fd, recv, rx);
                    if(ret != rx) {
                        printf("write failed ret[%d], rx[%d]\n", recv, rx);
                    } else {
                        cnts_wirte += ret;
                        //printf("write %d\n", ret);
                    }
                }
                #else
                cnts_wirte += rx;
                #endif
            } else {
                //printf("rx is %d\n", rx);
            }
        }
#if 1       
    } else {
        static bool flag = true;
        if(flag) {
            printf("over\n");
            close(fd);
            flag = false;
        }
    }
#endif    
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
