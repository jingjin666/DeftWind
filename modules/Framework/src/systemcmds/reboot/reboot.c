/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file reboot.c
 * Tool similar to UNIX reboot command
 */

#include <dp_config.h>
#include <dp_getopt.h>
#include <dp_task.h>
#include <dp_log.h>
#include <systemlib/systemlib.h>

__EXPORT int reboot_main(int argc, char *argv[]);

int reboot_main(int argc, char *argv[])
{
	int ch;
	bool to_bootloader = false;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = dp_getopt(argc, argv, "b", &myoptind, &myoptarg)) != -1) {
		switch (ch) {
		case 'b':
			to_bootloader = true;
			break;

		default:
			DP_ERR("usage: reboot [-b]\n"
				"   -b   reboot into the bootloader");

		}
	}

	dp_systemreset(to_bootloader);
}
