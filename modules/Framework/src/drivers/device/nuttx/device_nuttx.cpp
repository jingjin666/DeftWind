/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file device.cpp
 *
 * Fundamental driver base class for the device framework.
 */

#include "../device.h"
#include "dp_log.h"

#include <nuttx/arch.h>
#include <stdio.h>
#include <unistd.h>
#include <drivers/drv_device.h>

namespace device
{

/**
 * Interrupt dispatch table entry.
 */
struct irq_entry {
	int	irq;
	Device	*owner;
};

static const unsigned	irq_nentries = 8;		/**< size of the interrupt dispatch table */
static irq_entry	irq_entries[irq_nentries];	/**< interrupt dispatch table (XXX should be a vector) */

/**
 * Register an interrupt to a specific device.
 *
 * @param irq		The interrupt number to register.
 * @param owner		The device receiving the interrupt.
 * @return		OK if the interrupt was registered.
 */
static int	register_interrupt(int irq, Device *owner);

/**
 * Unregister an interrupt.
 *
 * @param irq		The previously-registered interrupt to be de-registered.
 */
static void	unregister_interrupt(int irq);

/**
 * Handle an interrupt.
 *
 * @param irq		The interrupt being invoked.
 * @param context	The interrupt register context.
 * @return		Always returns OK.
 */
static int	interrupt(int irq, void *context, void *arg);

Device::Device(const char *name,
	       int irq) :
	// public
	// protected
	_name(name),
	_debug_enabled(false),
	// private
	_irq(irq),
	_irq_attached(false)
{
	sem_init(&_lock, 0, 1);

	/* setup a default device ID. When bus_type is UNKNOWN the
	   other fields are invalid */
	_device_id.devid = 0;
	_device_id.devid_s.bus_type = DeviceBusType_UNKNOWN;
	_device_id.devid_s.bus = 0;
	_device_id.devid_s.address = 0;
	_device_id.devid_s.devtype = 0;
}

Device::~Device()
{
	sem_destroy(&_lock);

	if (_irq_attached) {
		unregister_interrupt(_irq);
	}
}

int
Device::init()
{
	int ret = OK;

	// If assigned an interrupt, connect it
	if (_irq) {
		/* ensure it's disabled */
		up_disable_irq(_irq);

		/* register */
		ret = register_interrupt(_irq, this);

		if (ret != OK) {
			goto out;
		}

		_irq_attached = true;
	}

out:
	return ret;
}

void
Device::interrupt_enable()
{
	if (_irq_attached) {
		up_enable_irq(_irq);
	}
}

void
Device::interrupt_disable()
{
	if (_irq_attached) {
		up_disable_irq(_irq);
	}
}

void
Device::interrupt(void *context)
{
	// default action is to disable the interrupt so we don't get called again
	interrupt_disable();
}

static int
register_interrupt(int irq, Device *owner)
{
	int ret = -ENOMEM;

	// look for a slot where we can register the interrupt
	for (unsigned i = 0; i < irq_nentries; i++) {
		if (irq_entries[i].irq == 0) {

			// great, we could put it here; try attaching it
			ret = irq_attach(irq, &interrupt, nullptr);

			if (ret == OK) {
				irq_entries[i].irq = irq;
				irq_entries[i].owner = owner;
			}

			break;
		}
	}

	return ret;
}

static void
unregister_interrupt(int irq)
{
	for (unsigned i = 0; i < irq_nentries; i++) {
		if (irq_entries[i].irq == irq) {
			irq_entries[i].irq = 0;
			irq_entries[i].owner = nullptr;
		}
	}
}

static int
interrupt(int irq, void *context, void *arg)
{
	for (unsigned i = 0; i < irq_nentries; i++) {
		if (irq_entries[i].irq == irq) {
			irq_entries[i].owner->interrupt(context);
			break;
		}
	}

	return OK;
}

int
Device::read(unsigned offset, void *data, unsigned count)
{
	return -ENODEV;
}

int
Device::write(unsigned offset, void *data, unsigned count)
{
	return -ENODEV;
}

int
Device::ioctl(unsigned operation, unsigned &arg)
{
	switch (operation) {
	case DEVIOCGDEVICEID:
		return (int)_device_id.devid;
	}

	return -ENODEV;
}

} // namespace device
