#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>

#include "Util.h"
#include "Scheduler.h"


namespace UAVRS{

uint8_t UAVRS::UAVRS_I2C::instance;
pthread_mutex_t UAVRS::UAVRS_I2C::instance_lock;

DeviceBus I2CDevice::businfo[I2CDevice::num_buses];


/*
  constructor for I2C wrapper class
 */    
UAVRS_I2C::UAVRS_I2C(uint8_t bus) :
  I2C(devname, devpath, map_bus_number(bus), 0, 100000UL)
{}

/*
  map ArduPilot bus numbers to UAVRS bus numbers
 */    
uint8_t UAVRS_I2C::map_bus_number(uint8_t bus) const
{
    // default to bus 1
    return 1;
}

/*
  implement wrapper for UAVRS I2C driver
 */
bool UAVRS_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len, bool split_transfers)
{
    set_address(address);
    if (!init_done) {
        if (pthread_mutex_lock(&instance_lock) != 0) {
            return false;
        }
        init_done = true;
        // we do late init() so we can setup the device paths
        
        snprintf(devname, sizeof(devname), "AP_I2C_%u", instance);
        snprintf(devpath, sizeof(devpath), "/dev/api2c%u", instance);
        init_ok = (init() == OK);
        if (init_ok) {
            instance++;
        }
        pthread_mutex_unlock(&instance_lock);
    }
    if (!init_ok) {
        return false;
    }

    if (split_transfers) {
        /*
          splitting the transfer() into two pieces avoids a stop condition
          with SCL low which is not supported on some devices (such as
          LidarLite blue label)
        */
        if (send && send_len) {
            if (transfer(send, send_len, nullptr, 0) != OK) {
                return false;
            }
        }
        if (recv && recv_len) {
            if (transfer(nullptr, 0, recv, recv_len) != OK) {
                return false;
            }
        }
    } else {
        // combined transfer
        if (transfer(send, send_len, recv, recv_len) != OK) {
            return false;
        }
    }
    return true;
}

I2CDevice::I2CDevice(uint8_t bus, uint8_t address) :
    _busnum(bus),
    _uavrsdev(_busnum),
    _address(address)
{
    set_device_bus(bus);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)bus, (unsigned)address);
    perf = perf_alloc(PC_ELAPSED, pname);
}


I2CDevice::~I2CDevice()
{
    printf("I2C device bus %u address 0x%02x closed\n", 
           (unsigned)_busnum, (unsigned)_address);
    perf_free(perf);
    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    perf_begin(perf);
    bool ret = _uavrsdev.do_transfer(_address, send, send_len, recv, recv_len, _split_transfers);
    perf_end(perf);
    return ret;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    return false;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    if (_busnum >= num_buses) {
        return nullptr;
    }
    struct DeviceBus &binfo = businfo[_busnum];
    return binfo.register_periodic_callback(period_usec, cb, this);
}

/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (_busnum >= num_buses) {
        return false;
    }

    struct DeviceBus &binfo = businfo[_busnum];

    return binfo.adjust_timer(h, period_usec);
}


AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    return dev;
}

}

