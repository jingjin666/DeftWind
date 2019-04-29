
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS

#include <assert.h>

#include "HAL_UAVRS_Class.h"
#include "AP_HAL_UAVRS_Private.h"

using namespace UAVRS;

#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"	// Usb Mavlink
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"	// Telecom MicroHard P900
#define UARTD_DEFAULT_DEVICE "/dev/ttyS6"	// Rtk com1 ouput position data
#define UARTB_DEFAULT_DEVICE "/dev/ttyS1"	// Rtk com2 input rtcm data and output raw position data
#define UARTE_DEFAULT_DEVICE "/dev/ttyS0"	// Backup Gps Ublox M8N
#define UARTF_DEFAULT_DEVICE "/dev/null"

static UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "APM_uartA");
static UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "APM_uartB");
static UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "APM_uartC");
static UARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "APM_uartD");
static UARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "APM_uartE");
static UARTDriver uartFDriver(UARTF_DEFAULT_DEVICE, "APM_uartF");


//static UARTDriver uartADriver;
//static UARTDriver uartBDriver;
//static UARTDriver uartCDriver;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;

HAL_UAVRS::HAL_UAVRS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        nullptr,            /* no uartD */
        nullptr,            /* no uartE */
        nullptr,            /* no uartF */
        nullptr,            /*i2c*/
        &spiDeviceManager,
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


void HAL_UAVRS::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    uartA->begin(115200);
    _member->init();

    callbacks->setup();
    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_UAVRS hal;
    return hal;
}

#endif
