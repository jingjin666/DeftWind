#include "SPIDevice.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "GPIO.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"


#define UAVRS_SPI_BUS_ADIS	4

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define UAVRS_SPIDEV_ADIS		1


FAR struct spi_dev_s *up_spiinitialize(int port){ return nullptr;}

/****************************************************************************
 * Name: up_spi_use_irq_save
 *
 * Description:
 *
 * return true if this bus needs to use irqsave/irqrestore for locking
 */   
FAR bool up_spi_use_irq_save(FAR struct spi_dev_s *){ return false;}

/****************************************************************************
 * Name: up_spi_set_need_irq_save
 *
 * Description:
 *
 * set a bus to needing irqsave/irqrestore for locking
 */   
FAR void up_spi_set_need_irq_save(FAR struct spi_dev_s *){}

irqstate_t  irqsave(void){ return 0;}

void irqrestore(irqstate_t){}



namespace UAVRS {

#define MHZ (1000U*1000U)
#define KHZ (1000U)

SPIDesc SPIDeviceManager::device_table[] = {
    SPIDesc("adis16375",	UAVRS_SPI_BUS_ADIS, (spi_devtype_e)UAVRS_SPIDEV_ADIS, SPIDEV_MODE3, 500*KHZ, 20*MHZ),
    SPIDesc(nullptr, 0, (spi_devtype_e)0, (spi_mode_e)0, 0, 0),
};

SPIDevice::SPIDevice(SPIBus &_bus, SPIDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
    set_device_bus(_bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);
    SPI_SELECT(bus.dev, device_desc.device, false);
    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name,
             (unsigned)bus.bus, (unsigned)device_desc.device);
    perf = perf_alloc(PC_ELAPSED, pname);
    printf("SPI device %s on %u:%u at speed %u mode %u\n",
           device_desc.name,
           (unsigned)bus.bus, (unsigned)device_desc.device,
           (unsigned)frequency, (unsigned)device_desc.mode);
}

SPIDevice::~SPIDevice()
{
    printf("SPI device %s on %u:%u closed\n", device_desc.name,
           (unsigned)bus.bus, (unsigned)device_desc.device);
    perf_free(perf);
    free(pname);
}


bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        frequency = device_desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        frequency = device_desc.lowspeed;
        break;
    }
    return true;
}


/*
  low level transfer function
 */
void SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    /*
      to accomodate the method in UAVRS drivers of using interrupt
      context for SPI device transfers we need to check if PX4 has
      registered a driver on this bus.  If not then we can avoid the
      irqsave/irqrestore and get bus parallelism for DMA enabled
      buses.

      There is a race in this if a UAVRS driver starts while we are
      running this, but that would only happen at early boot and is
      very unlikely

      yes, this is a nasty hack. Suggestions for a better method
      appreciated.
     */
    bool use_irq_save = up_spi_use_irq_save(bus.dev);
    irqstate_t state;
    if (use_irq_save) {
        state = irqsave();
    }
    perf_begin(perf);
    SPI_LOCK(bus.dev, true);
    SPI_SETFREQUENCY(bus.dev, frequency);
    SPI_SETMODE(bus.dev, device_desc.mode);
    SPI_SETBITS(bus.dev, 8);
    SPI_SELECT(bus.dev, device_desc.device, true);
    SPI_EXCHANGE(bus.dev, send, recv, len);
    if (!cs_forced) {
        SPI_SELECT(bus.dev, device_desc.device, false);
    }
    SPI_LOCK(bus.dev, false);
    perf_end(perf);
    if (use_irq_save) {
        irqrestore(state);
    }
}


bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (send_len == recv_len && send == recv) {
        // simplest cases, needed for DMA
        do_transfer(send, recv, recv_len);
        return true;
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    do_transfer(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    uint8_t buf[len];
    memcpy(buf, send, len);
    do_transfer(buf, buf, len);
    memcpy(recv, buf, len);
    return true;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}


/*
  allow for control of SPI chip select pin
 */
bool SPIDevice::set_chip_select(bool set)
{
    cs_forced = set;
    SPI_SELECT(bus.dev, device_desc.device, set);
    return true;
}

/*
  return a SPIDevice given a string device name
 */
AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; device_table[i].name; i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (device_table[i].name == nullptr) {
        printf("SPI: Invalid device name: %s\n", name);
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }

    SPIDesc &desc = device_table[i];

    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new SPIBus;
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;
        busp->dev = up_spiinitialize(desc.bus);
        buses = busp;
    }

    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*busp, desc));
}

}
