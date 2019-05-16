
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS

#include <assert.h>
#include <stdio.h>

#include <drivers/drv_hrt.h>

#include "HAL_UAVRS_Class.h"
#include "AP_HAL_UAVRS_Private.h"

using namespace UAVRS;

#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"	// Usb Mavlink
#define UARTB_DEFAULT_DEVICE "/dev/ttyS1"	// Rtk com2 input rtcm data and output raw position data
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"	// Rtk com1 ouput position data
#define UARTD_DEFAULT_DEVICE "/dev/ttyS3"	// Backup Peripheral usart
#define UARTE_DEFAULT_DEVICE "/dev/ttyS4"	// Backup Peripheral usart
#define UARTF_DEFAULT_DEVICE "/dev/ttyS5"   // Telecom MicroHard P900
#define UARTG_DEFAULT_DEVICE "/dev/ttyS7"   // Backup Gps Ublox M8N

static UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "UAVRS_uartA");
static UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "UAVRS_uartB");
static UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "UAVRS_uartC");
static UARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "UAVRS_uartD");
static UARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "UAVRS_uartE");
static UARTDriver uartFDriver(UARTF_DEFAULT_DEVICE, "UAVRS_uartF");
static UARTDriver uartGDriver(UARTG_DEFAULT_DEVICE, "UAVRS_uartG");

static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;

static UAVRS::I2CDeviceManager i2c_mgr_instance;
static UAVRS::SPIDeviceManager spi_mgr_instance;



extern const AP_HAL::HAL& hal;


HAL_UAVRS::HAL_UAVRS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,            
        &uartEDriver,            
        &uartFDriver,
        &uartGDriver,
        &i2c_mgr_instance,            /*i2c*/
        &spi_mgr_instance,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        nullptr)
{}

bool _uavrs_thread_should_exit = false;        /**< Daemon exit flag */
static bool thread_running = false;        /**< Daemon status flag */
static int daemon_task;                /**< Handle of daemon task / thread */
bool uavrs_ran_overtime;

/*
  set the priority of the main APM task
 */
void hal_px4_set_priority(uint8_t priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    sched_setscheduler(daemon_task, SCHED_FIFO, &param);    
}

/*
  this is called when loop() takes more than 1 second to run. If that
  happens then something is blocking for a long time in the main
  sketch - probably waiting on a low priority driver. Set the priority
  of the APM task low to let the driver run.
 */
static void loop_overtime(void *)
{
    hal_px4_set_priority(UAVRS_OVERTIME_PRIORITY);
    uavrs_ran_overtime = true;
}

static AP_HAL::HAL::Callbacks* g_callbacks;

static int main_loop(int argc, char **argv)
{
    hal.uartA->begin(115200);
    hal.uartB->begin(115200);
    hal.uartC->begin(115200);
    hal.uartD->begin(115200);
    hal.uartE->begin(115200);
    hal.uartF->begin(115200);
    hal.uartG->begin(115200);

    hal.scheduler->init();

    // init the I2C wrapper class
    UAVRS_I2C::init_lock();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_px4_set_priority(UAVRS_MAIN_PRIORITY);

    schedulerInstance.hal_initialized();

    g_callbacks->setup();
    hal.scheduler->system_initialized();

    perf_counter_t perf_loop = perf_alloc(PC_ELAPSED, "APM_loop");
    perf_counter_t perf_overrun = perf_alloc(PC_COUNT, "APM_overrun");
    struct hrt_call loop_overtime_call;

    thread_running = true;

    /*
      switch to high priority for main loop
     */
    hal_px4_set_priority(UAVRS_MAIN_PRIORITY);

    while (!_uavrs_thread_should_exit) {
        perf_begin(perf_loop);
        
        /*
          this ensures a tight loop waiting on a lower priority driver
          will eventually give up some time for the driver to run. It
          will only ever be called if a loop() call runs for more than
          0.1 second
         */
        hrt_call_after(&loop_overtime_call, 100000, (hrt_callout)loop_overtime, nullptr);

        g_callbacks->loop();

        if (uavrs_ran_overtime) {
            /*
              we ran over 1s in loop(), and our priority was lowered
              to let a driver run. Set it back to high priority now.
             */
            hal_px4_set_priority(UAVRS_MAIN_PRIORITY);
            perf_count(perf_overrun);
            uavrs_ran_overtime = false;
        }

        perf_end(perf_loop);

        /*
          give up 250 microseconds of time, to ensure drivers get a
          chance to run. This relies on the accurate semaphore wait
          using hrt in semaphore.cpp
         */
        hal.scheduler->delay_microseconds(250);
    }
    thread_running = false;
    return 0;
}

static void usage(void)
{
    printf("Usage: %s [options] {start,stop,status}\n", SKETCHNAME);
    printf("Options:\n");
    printf("\t-d  DEVICE         set terminal device (default %s)\n", UARTA_DEFAULT_DEVICE);
    printf("\t-d2 DEVICE         set second terminal device (default %s)\n", UARTC_DEFAULT_DEVICE);
    printf("\t-d3 DEVICE         set 3rd terminal device (default %s)\n", UARTD_DEFAULT_DEVICE);
    printf("\t-d4 DEVICE         set 2nd GPS device (default %s)\n", UARTE_DEFAULT_DEVICE);
    printf("\n");
}

void HAL_UAVRS::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    int i;

    if (argc < 1) {
        printf("%s: missing command (try '%s start')", 
               SKETCHNAME, SKETCHNAME);
        usage();
        exit(1);
    }

    g_callbacks = callbacks;

    for (i=0; i<argc; i++) {
        if (strcmp(argv[i], "start") == 0) {
            if (thread_running) {
                printf("%s already running\n", SKETCHNAME);
                /* this is not an error */
                exit(0);
            }

            _uavrs_thread_should_exit = false;
            daemon_task = dp_task_create(SKETCHNAME,
                                             SCHED_FIFO,
                                             UAVRS_MAIN_PRIORITY,
                                             UAVRS_MAIN_THREAD_STACK_SIZE,
                                             main_loop,
                                             nullptr);
            exit(0);
        }

        if (strcmp(argv[i], "stop") == 0) {
            _uavrs_thread_should_exit = true;
            exit(0);
        }
 
        if (strcmp(argv[i], "status") == 0) {
            if (_uavrs_thread_should_exit && thread_running) {
                printf("\t%s is exiting\n", SKETCHNAME);
            } else if (thread_running) {
                printf("\t%s is running\n", SKETCHNAME);
            } else {
                printf("\t%s is not started\n", SKETCHNAME);
            }
            exit(0);
        }
    }
    
    usage();
    exit(1);
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_UAVRS hal;
    return hal;
}

#endif
