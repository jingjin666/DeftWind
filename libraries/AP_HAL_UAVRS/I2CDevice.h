/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <drivers/device/i2c.h>
#include "Device.h"



namespace UAVRS {

/*
  wrapper class for I2C to expose protected functions from Firmware
 */
class UAVRS_I2C : public device::I2C {
public:
    UAVRS_I2C(uint8_t bus);
    bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len, bool split_transfers);

    void set_retries(uint8_t retries) {
        _retries = retries;
    }

    uint8_t map_bus_number(uint8_t bus) const;

    // setup instance_lock
    static void init_lock(void) {
        pthread_mutex_init(&instance_lock, nullptr);
    }

private:
    static uint8_t instance;
    static pthread_mutex_t instance_lock;
    bool init_done;
    bool init_ok;
    char devname[10];
    char devpath[14];
};


class I2CDevice : public AP_HAL::I2CDevice {
public:
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
            return static_cast<I2CDevice*>(dev);
    }
    I2CDevice(uint8_t bus, uint8_t address);
    ~I2CDevice();

    /* AP_HAL::I2CDevice implementation */

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override {_uavrsdev.set_retries(retries); }


    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times);


    /* See AP_HAL::Device::set_speed() */
    bool set_speed(enum AP_HAL::Device::Speed speed) override { return true; }

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() {
        // if asking for invalid bus number use bus 0 semaphore
        return &businfo[_busnum<num_buses?_busnum:0].semaphore;
    }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    void set_split_transfers(bool set) override {
        _split_transfers = set;
    }

private:
    static const uint8_t num_buses = 2;
    static DeviceBus businfo[num_buses];

    uint8_t _busnum;
    UAVRS_I2C _uavrsdev;
    uint8_t _address;
    perf_counter_t perf;
    char *pname;
    bool _split_transfers;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) override;
};

}
