/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_i2c.c
 * Tests main file, loads individual tests.
 *
 * @author User <mail@example.com>
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dp_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <arch/board/board.h>

#include "tests.h"

#include <drivers/device/i2c.h>

class DP_I2C : public device::I2C {
public:
	DP_I2C(uint8_t bus);
	bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len, bool split_transfers);

	void set_retries(uint8_t retries) {
		_retries = retries;
	}

	uint8_t map_bus_number(uint8_t bus) const;
#if 0
	// setup instance_lock
	static void init_lock(void) {
		pthread_mutex_init(&instance_lock, nullptr);
	}
#endif	
private:
	static uint8_t instance;
	//static pthread_mutex_t instance_lock;
	bool init_done;
	bool init_ok;
	char devname[10];
	char devpath[14];

};

uint8_t DP_I2C::instance;
//pthread_mutex_t DP_I2C::instance_lock;

/*
  constructor for I2C wrapper class
 */    
DP_I2C::DP_I2C(uint8_t bus) : I2C(devname, devpath, map_bus_number(bus), 0, 400000UL)
{
	_debug_enabled = true;
	printf("DP_I2C Constructor.\n");
}

/*
  map bus numbers
 */    
uint8_t DP_I2C::map_bus_number(uint8_t bus) const
{
	switch (bus) {
	case 1:
		// map to expansion bus
		return 1;

	case 2:
		// map to expansion bus
		return 2;

	case 3:
		// map to expansion bus
		return 3;
		
	case 4:
		// map to expansion bus
	return 4;
	}
	// default to bus 1
	return 1;
}

/*
  implement wrapper for DP I2C driver
 */
bool DP_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len, bool split_transfers)
{
	set_address(address);
	if (!init_done) {
		#if 0
		if (pthread_mutex_lock(&instance_lock) != 0) {
			return false;
		}
		#endif
		init_done = true;
		// we do late init() so we can setup the device paths
		
		snprintf(devname, sizeof(devname), "DP_I2C_%u", instance);
		snprintf(devpath, sizeof(devpath), "/dev/dpi2c%u", instance);
		init_ok = (init() == OK);
		if (init_ok) {
			instance++;
		}
		#if 0
		pthread_mutex_unlock(&instance_lock);
		#endif
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

class I2CDevice {
public:
	I2CDevice(uint8_t bus, uint8_t address, bool split_transfers);
	~I2CDevice();
	bool transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);
private:
	uint8_t _busnum;
	DP_I2C _dpdev;
	uint8_t _address;
	bool _split_transfers;
};

I2CDevice::I2CDevice(uint8_t bus, uint8_t address, bool split_transfers):
	_busnum(bus),
	_dpdev(_busnum),
	_address(address),
	_split_transfers(split_transfers)
{
	printf("I2CDevice Constructor.\n");
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    bool ret = _dpdev.do_transfer(_address, send, send_len, recv, recv_len, _split_transfers);
    return ret;
}

I2CDevice *i2c_dev;

/****************************************************************************
 * Name: test_usart
 ****************************************************************************/

__BEGIN_DECLS

int test_i2c(int argc, char *argv[])
{
	const uint8_t send[2] = {1, 2};
	uint8_t recv[2] = {0};
	bool ret;
	uint8_t bus = 0;
	
	printf("Test i2c driver.\n");

	if(argc < 2) {
		printf("Invalid Param::Please tests spi 1|2|3|4.\n");
		return -1;
	}
	if(argv[1] != NULL) {
		if((!strcmp(argv[1], "1")) ||(!strcmp(argv[1], "2")) ||(!strcmp(argv[1], "3")) || (!strcmp(argv[1], "4"))) {
			bus = atoi(argv[1]);
		} else {
			printf("Invalid Param::Please tests i2c 1|2|3|4.\n");
			return -1;
		}
	}else {
		printf("Invalid Param::Please tests i2c 1|2|3|4\n");
		return -1;
	}
	
	if(i2c_dev == nullptr) {
		i2c_dev = new I2CDevice(bus, 0x50, false);
	}
	
	for(uint8_t i=0; i < 5; i++) {
		ret = i2c_dev->transfer(send, sizeof(send), recv, sizeof(recv));
		if(ret) {
			printf("I2c transfer [%d] complete.\n", i);
			printf("%d, %d\n", recv[0], recv[1]);
		} else {
			printf("I2c transfer [%d] error.\n", i);
		}
		sleep(1);
	}
	return 0;
}
__END_DECLS
