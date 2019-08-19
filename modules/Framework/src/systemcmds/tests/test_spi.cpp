/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_spi.c
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

#include <drivers/device/spi.h>

#define MHZ (1000U*1000U)
#define KHZ (1000U)

class DP_SPI : public device::SPI {
public:
	DP_SPI(uint8_t bus, 
		enum spi_devtype_e device,
	   	enum spi_mode_e mode,
	    	uint32_t frequency);
	bool transfer_singleduplex(uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);
	bool transfer_fullduplex(uint8_t *send, uint8_t *recv, uint32_t len);

	uint8_t map_bus_number(uint8_t bus) const;

private:
	static uint8_t instance;
	bool init_done;
	bool init_ok;
	char devname[10];
	char devpath[14];
};

uint8_t DP_SPI::instance;

/*
  constructor for SPI wrapper class
 */    
DP_SPI::DP_SPI(uint8_t bus, 
		enum spi_devtype_e device,
	   	enum spi_mode_e mode,
	    	uint32_t frequency) : SPI(devname, devpath, map_bus_number(bus), device, mode, frequency)
{
	_debug_enabled = true;
	printf("DP_SPI Constructor.\n");
}

/*
  map bus numbers
 */    
uint8_t DP_SPI::map_bus_number(uint8_t bus) const
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


bool DP_SPI::transfer_singleduplex(uint8_t *send, uint32_t send_len,
					 uint8_t *recv, uint32_t recv_len)
{
	if (!init_done) {
		init_done = true;
		// we do late init() so we can setup the device paths
		
		snprintf(devname, sizeof(devname), "DP_SPI_%u", instance);
		snprintf(devpath, sizeof(devpath), "/dev/dpspi%u", instance);
		init_ok = (init() == OK);
		if (init_ok) {
			instance++;
		}
	}
	if (!init_ok) {
		return false;
	}

	if (send_len == recv_len && send == recv) {
	    // simplest cases, needed for DMA
	    transfer(send, recv, recv_len);
	    return true;
	}
	uint8_t buf[send_len+recv_len];
	if (send_len > 0) {
	    memcpy(buf, send, send_len);
	}
	if (recv_len > 0) {
	    memset(&buf[send_len], 0, recv_len);
	}
	transfer(buf, buf, send_len+recv_len);
	if (recv_len > 0) {
	    memcpy(recv, &buf[send_len], recv_len);
	}
	
	return true;
}

/*
  implement wrapper for DP SPI driver
 */
bool DP_SPI::transfer_fullduplex(uint8_t *send, uint8_t *recv, uint32_t len)
{
	if (!init_done) {
		init_done = true;
		// we do late init() so we can setup the device paths
		
		snprintf(devname, sizeof(devname), "DP_SPI_%u", instance);
		snprintf(devpath, sizeof(devpath), "/dev/dpspi%u", instance);
		init_ok = (init() == OK);
		if (init_ok) {
			instance++;
		}
	}
	if (!init_ok) {
		return false;
	}

	// combined transfer
	if (transfer(send, recv, len) != OK) {
		return false;
	}
	
	return true;
}

DP_SPI *spi_dev;

/****************************************************************************
 * Name: test_usart
 ****************************************************************************/

__BEGIN_DECLS

static void fifo_dump(uint8_t *fifo, int size, int per)
{
    for(int k = 0; k < size; k++) {
        if((k!=0) && (k%per == 0))
            printf("\r\n");

        printf("0x%02x, ", fifo[k]);
    }
    printf("\r\n");
}

static void adi16375_test(DP_SPI *dev)
{
    uint8_t send[2] = {0x7E, 0};
    uint8_t recv[2] = {0};

    if(dev == nullptr) {
		dev = new DP_SPI(UAVRS_SPI_BUS_ADIS, (enum spi_devtype_e)UAVRS_SPIDEV_ADIS, SPIDEV_MODE3, 1*MHZ);
	}
}


/* 关于 transfer_fullduplex 和 transfer_singleduplex
 * single多一层内存拷贝, full少一层内存拷贝，但是需要注意返回数据是从第二个字节开始，第一个字节为cmd。
 * 也可以先发一个cmd，CS不要拉高，然后再full接收
 */
#define SIMPLE 16
static void mpu9250_test(DP_SPI *dev)
{
    bool ret;
    uint8_t send[SIMPLE * 14] = {0};
    uint8_t recv[SIMPLE * 14] = {0};
    uint16_t fifo_size = 0;
    
    if(dev == nullptr) {
		dev = new DP_SPI(UAVRS_SPI_BUS_MPU_9250, (enum spi_devtype_e)UAVRS_SPIDEV_MPU_9250, SPIDEV_MODE3, 20*MHZ);
	}
    
    send[0] = 0xf5;
    ret = dev->transfer_fullduplex(send, send, 2);
    if(ret)
        printf("VENDOR ID is 0x%02x, 0x%02x\n", send[0], send[1]);
    else
        printf("Spi transfer error\n");

    for(uint8_t i=0; i < 200; i++) {
        memset(send, 0, sizeof(send));
        memset(recv, 0, sizeof(recv));
    
        send[0] = 0x72 | 0x80;
        #if 0
        ret = dev->transfer_fullduplex(send, send, 3);
        //printf("FIFO CNTS is 0x%02x, 0x%02x, 0x%02x\n", send[0], send[1], send[2]);
        fifo_size = (send[1]<<8) | send[2];
        #else
        ret = dev->transfer_singleduplex(send, 1, recv, 2);
        printf("FIFO CNTS is 0x%02x, 0x%02x\n", recv[0], recv[1]);
        fifo_size = (recv[0]<<8) | recv[1];
        #endif
        printf("FIFO SIZE is %d\n", fifo_size);

        memset(send, 0, sizeof(send));
        memset(recv, 0, sizeof(recv));

    #if 0
        // 直接读寄存器
        send[0] = 0x3b | 0x80;
        ret = dev->transfer_fullduplex(send, send, 14);
        if(ret) {
            memcpy(recv, &send[1], 14);
            fifo_dump(recv, 14, 14);
        }
    #endif

    #if 1
        // 读取FIFO
        send[0] = 0x74 | 0x80;
        #if 1
        ret = dev->transfer_fullduplex(send, send, 3*14);
        if(ret) {
            memcpy(recv, &send[1], 3*14);
            fifo_dump(recv, 3*14, 14);
        }
        #else
        ret = dev->transfer_singleduplex(send, 1, recv, sizeof(recv));
        if(ret) {
            fifo_dump(recv, sizeof(recv), 14);
        }
        #endif
   #endif     
        sleep(1);
    }
}

int test_spi(int argc, char *argv[])
{
	bool ret;
	uint8_t bus = 0;
	
	printf("Test spi driver.\n"); 

	if(argc < 2) {
		printf("Invalid Param::Please tests spi 1|2|3|4\n");
		return -1;
	}

	if(argv[1] != NULL) {
		if((!strcmp(argv[1], "1")) || (!strcmp(argv[1], "2")) || (!strcmp(argv[1], "3")) || (!strcmp(argv[1], "4"))) {
			bus = atoi(argv[1]);
		} else {
			printf("Invalid Param::Please tests spi 1|2|3|4\n");
			return -1;
		}
	}else {
		printf("Invalid Param::Please tests spi 1|2|3|4\n");
		return -1;
	}

    if(bus == UAVRS_SPI_BUS_MPU_9250) {
        printf("mpu9250_test bus %d\n", bus);
        mpu9250_test(spi_dev);
    }
    
	return 0;
}
__END_DECLS
