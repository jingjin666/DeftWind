/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file top.c
 * Tool similar to UNIX top command
 * @see http://en.wikipedia.org/wiki/Top_unix
 *
 * @author Lorenz Meier <lorenz@dp.io>
 */

#include <dp_config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <systemlib/cpuload.h>
#include <systemlib/printload.h>
#include <drivers/drv_hrt.h>

/**
 * Start the top application.
 */
__EXPORT int top_main(int argc, char *argv[]);

int
top_main(int argc, char *argv[])
{
	hrt_abstime curr_time = hrt_absolute_time();

	struct print_load_s load;
	init_print_load_s(curr_time, &load);

	/* clear screen */
	dprintf(1, "\033[2J\n");

	if (argc > 1 && !strcmp(argv[1], "once")) {
		print_load(curr_time, 1, &load);
		sleep(1);
		print_load(hrt_absolute_time(), 1, &load);
		return 0;
	}

	for (;;) {
		print_load(curr_time, 1, &load);

		/* Sleep 200 ms waiting for user input five times ~ 1s */
		for (int k = 0; k < 5; k++) {
			char c;

			struct pollfd fds;
			int ret;
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				read(0, &c, 1);

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return OK;
					/* not reached */
				}
			}

			usleep(200000);
		}

		curr_time = hrt_absolute_time();
	}

	return OK;
}
