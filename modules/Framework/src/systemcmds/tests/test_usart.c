/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_usart.c
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

#include <math.h>
#include <float.h>

/****************************************************************************
 * Name: test_usart
 ****************************************************************************/

int test_usart(int argc, char *argv[])
{
	/* input handling */
	char *uart_name = "/dev/ttyS0";

	if (argc > 1) { uart_name = argv[1]; }

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int test_uart = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (test_uart < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return test_uart;

	} else {
		printf("Writing to UART %s\n", uart_name);
	}

	char sample_test_uart[100] = {0};

	int i,nbytes;

	for (i = 0; i < 5; i++) {
		nbytes = sprintf(sample_test_uart, "This is a usart test smaple. #%d\n", i);
		write(test_uart, sample_test_uart, nbytes);
		sleep(1);
	}

	printf("Wrote %d bytes on UART %s\n", nbytes*i, uart_name);

	close(test_uart);

	return 0;
}
