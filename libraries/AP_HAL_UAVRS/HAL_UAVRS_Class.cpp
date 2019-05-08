
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS

#include <assert.h>
#include <stdio.h>

#include "HAL_UAVRS_Class.h"
#include "AP_HAL_UAVRS_Private.h"

using namespace UAVRS;

#define UARTA_DEFAULT_DEVICE "/dev/ttyACM0"	// Usb Mavlink
#define UARTB_DEFAULT_DEVICE "/dev/ttyS0"	// Telecom MicroHard P900
#define UARTC_DEFAULT_DEVICE "/dev/ttyS2"	// Rtk com1 ouput position data
#define UARTD_DEFAULT_DEVICE "/dev/ttyS3"	// Rtk com2 input rtcm data and output raw position data
#define UARTE_DEFAULT_DEVICE "/dev/ttyS4"	// Backup Gps Ublox M8N
#define UARTF_DEFAULT_DEVICE "/dev/null"

static UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "UAVRS_uartA");
static UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "UAVRS_uartB");
static UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "UAVRS_uartC");
static UARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "UAVRS_uartD");
static UARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "UAVRS_uartE");
static UARTDriver uartFDriver(UARTF_DEFAULT_DEVICE, "UAVRS_uartF");

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
        &uartDDriver,            /* no uartD */
        &uartEDriver,            /* no uartE */
        &uartFDriver,            /* no uartF */
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


void HAL_UAVRS::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    hal.uartA->begin(115200);
    hal.uartB->begin(115200);
    hal.uartC->begin(115200);
    _member->init();

    // init the I2C wrapper class
    UAVRS_I2C::init_lock();

    const uint8_t send[] = {0};
	uint8_t recv[64] = {0};
	bool ret;
	uint8_t bus = 0;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    _dev = hal.i2c_mgr->get_device(bus, 0xA0);

    callbacks->setup();

    schedulerInstance.hal_initialized();

    scheduler->system_initialized();

    for (;;) {
        callbacks->loop();

        ret = _dev->transfer(send, 0, recv, sizeof(recv));

        if(ret) {
			for(uint8_t i = 0; i < sizeof(recv); i++) {
				printf("0x%02X ", recv[i]);
				if((i + 1) % 16 == 0) {
					printf("\n");
				}
			}
			printf("\n");
		} else {
			printf("I2c transfer error.\n");
		}
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_UAVRS hal;
    return hal;
}

#endif
