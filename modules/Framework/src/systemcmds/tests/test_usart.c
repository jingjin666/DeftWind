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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
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
    char sample_test_uart[512] = {0};

	if (argc > 1) { uart_name = argv[1]; }

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int test_uart_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (test_uart_fd < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return test_uart_fd;
	} else {
		printf("R/W test to UART %s\n", uart_name);
	}

#ifdef NORMAL_READ_TEST
	int i,nbytes;

	for (i = 0; i < 5; i++) {
		nbytes = sprintf(sample_test_uart, "This is a usart test smaple. #%d\n", i);
        printf("write......\n");
		write(test_uart_fd, sample_test_uart, nbytes);
		sleep(1);
	}

	printf("Wrote %d bytes on UART %s\n", nbytes*i, uart_name);

	close(test_uart_fd);
#endif

    /* LOOPBACK TEST */
    uint32_t nread = 0, nwrite = 0;
    static uint32_t rx_cnts = 0;
    while(1) {
        usleep(1000);
        if (ioctl(test_uart_fd, FIONREAD, (unsigned long)&nread) == 0) {
            if (nread > sizeof(sample_test_uart)) {
                nread = sizeof(sample_test_uart);
            }

            if (nread > 0) {
                int ret = read(test_uart_fd, sample_test_uart, nread);
                if(ret > 0) {
                    memset(&sample_test_uart[ret], 0, sizeof(sample_test_uart)-ret);
                    #if 0
                    rx_cnts += ret;
                    printf("ret is %d , rx_cnts:: %d\n", ret, rx_cnts);
                    printf("sample_test_uart:: %s\n", sample_test_uart);
                    #else
                    if (ioctl(test_uart_fd, FIONSPACE, (unsigned long)&nwrite) == 0) {
                        if (nwrite > ret) {
                            nwrite = ret;
                        } else {
                            printf("no avalible send space %d <= %d\n", nwrite, ret);
                        }
                        if (nwrite > 0) {
                            ret = write(test_uart_fd, sample_test_uart, nwrite);
                        }
                    }
                    #endif
                }
            }
        }
    }

    return 0;
}
