/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_led.c
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

#include <drivers/drv_led.h>

#include "tests.h"

/****************************************************************************
 * Name: test_usart
 ****************************************************************************/

int test_led(int argc, char *argv[])
{
	int fd;
	
	printf("Test led driver.\n");
	
	drv_led_start();

	fd = open(LED0_DEVICE_PATH, 0);
	if(fd == -1) {
		printf("Failed to Open device[%s]\n", LED0_DEVICE_PATH);
		return -1;
	}

	if(argc < 2) {
		printf("Invalid Param::Please tests led on |off.\n");
		return -1;
	}

	if(argv[1] != NULL) {
		if(strcmp(argv[1], "on") == 0) {
			ioctl(fd, LED_ON, 1);
			printf("Led on.\n");
		} else if(strcmp(argv[1], "off") == 0) {
			ioctl(fd, LED_OFF, 1);
			printf("Led off.\n");
		} else {
			printf("Invalid Param::Please tests led on |off.\n");
			return -1;
		}
	} else {
		printf("Param error.\n");
		return -1;
	}

	close(fd);
	
	return 0;
}
