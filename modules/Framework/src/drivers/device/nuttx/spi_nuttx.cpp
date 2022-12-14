/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include <nuttx/arch.h>

#include "../spi.h"

#ifndef CONFIG_SPI_EXCHANGE
# error This driver requires CONFIG_SPI_EXCHANGE
#endif

namespace device
{

SPI::SPI(const char *name,
	 const char *devname,
	 int bus,
	 enum spi_devtype_e device,
	 enum spi_mode_e mode,
	 uint32_t frequency,
	 int irq) :
	// base class
	CDev(name, devname, irq),
	// public
	// protected
	locking_mode(LOCK_PREEMPTION),
	// private
	_device(device),
	_mode(mode),
	_frequency(frequency),
	_dev(nullptr),
	_bus(bus)
{
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	// XXX no way to let go of the bus...
}

int
SPI::init()
{
	int ret = OK;

	/* attach to the spi bus */
	if (_dev == nullptr) {
		_dev = dp_spibus_initialize(_bus);
	}

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init SPI");
		ret = -ENOENT;
		goto out;
	}

	// tell other SPI users that we may be doing transfers in
	// interrupt context
	//up_spi_set_need_irq_save(_dev);

	/* deselect device to ensure high to low transition of pin select */
	SPI_SELECT(_dev, _device, false);

	/* call the probe function to check whether the device is present */
	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", _bus, _device, _frequency / 1000);

out:
	return ret;
}

int
SPI::probe()
{
	// assume the device is too stupid to be discoverable
	return OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			irqstate_t state = up_irq_save();
			result = _transfer(send, recv, len);
			up_irq_restore(state);
		}
		break;

	case LOCK_THREADS:
		SPI_LOCK(_dev, true);
		result = _transfer(send, recv, len);
		SPI_LOCK(_dev, false);
		break;

	case LOCK_NONE:
		result = _transfer(send, recv, len);
		break;
	}

	return result;
}

void
SPI::set_frequency(uint32_t frequency)
{
	_frequency = frequency;
}

int
SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
    SPI_LOCK(_dev, true);
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 8);
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);
    SPI_LOCK(_dev, false);
	return OK;
}

} // namespace device
