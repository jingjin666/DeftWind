/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file i2c.h
 *
 * Base class for devices connected via I2C.
 */

#ifndef _DEVICE_I2C_H
#define _DEVICE_I2C_H

#include "../device.h"

#include <dp_i2c.h>

namespace device __EXPORT
{

/**
 * Abstract class for character device on I2C
 */
class __EXPORT I2C : public CDev
{

public:

	/**
	 * Get the address
	 */
	int16_t		get_address() const { return _address; }

	static int	set_bus_clock(unsigned bus, unsigned clock_hz);

	static unsigned	int	_bus_clocks[3];

protected:
	/**
	 * The number of times a read or write operation will be retried on
	 * error.
	 */
	unsigned		_retries;

	/**
	 * The I2C bus number the device is attached to.
	 */
	int			_bus;

	/**
	 * @ Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		I2C bus on which the device lives
	 * @param address	I2C bus address, or zero if set_address will be used
	 * @param frequency	I2C bus frequency for the device (currently not used)
	 * @param irq		Interrupt assigned to the device (or zero if none)
	 */
	I2C(const char *name,
	    const char *devname,
	    int bus,
	    uint16_t address,
	    uint32_t frequency,
	    int irq = 0);
	virtual ~I2C();

	virtual int	init();

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int	probe();

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(const uint8_t *send, unsigned send_len,
				 uint8_t *recv, unsigned recv_len);

	/**
	 * Perform a multi-part I2C transaction to the device.
	 *
	 * @param msgv		An I2C message vector.
	 * @param msgs		The number of entries in the message vector.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(dp_i2c_msg_t *msgv, unsigned msgs);

	/**
	 * Change the bus address.
	 *
	 * Most often useful during probe() when the driver is testing
	 * several possible bus addresses.
	 *
	 * @param address	The new bus address to set.
	 */
	void		set_address(uint16_t address)
	{
		_address = address;
		_device_id.devid_s.address = _address;
	}

private:
	uint16_t		_address;
	uint32_t		_frequency;
	dp_i2c_dev_t		*_dev;

	I2C(const device::I2C &);
	I2C operator=(const device::I2C &);
};

} // namespace device

#endif /* _DEVICE_I2C_H */
