/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mixer.c
 *
 * Mixer utility.
 */

#include <dp_config.h>
#include <dp_posix.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>

#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <uORB/topics/actuator_controls.h>

/**
 * Mixer utility for loading mixer files to devices
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mixer_main(int argc, char *argv[]);

static void	usage(const char *reason);
static int	load(const char *devname, const char *fname);

int
mixer_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "load")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = load(argv[2], argv[3]);

		if (ret != 0) {
			warnx("failed to load mixer");
			return 1;
		}

	} else {
		usage("Unknown command");
		return 1;
	}

	return 0;
}

static void
usage(const char *reason)
{
	if (reason && *reason) {
		DP_INFO("%s", reason);
	}

	DP_INFO("usage:");
	DP_INFO("  mixer load <device> <filename>");
}

static int
load(const char *devname, const char *fname)
{
	// sleep a while to ensure device has been set up
	usleep(20000);

	int dev;

	/* open the device */
	if ((dev = dp_open(devname, 0)) < 0) {
		warnx("can't open %s\n", devname);
		return 1;
	}

	/* reset mixers on the device */
	if (dp_ioctl(dev, MIXERIOCRESET, 0)) {
		warnx("can't reset mixers on %s", devname);
		return 1;
	}

	char buf[2048];

	if (load_mixer_file(fname, &buf[0], sizeof(buf)) < 0) {
		warnx("can't load mixer: %s", fname);
		return 1;
	}

	/* Pass the buffer to the device */
	int ret = dp_ioctl(dev, MIXERIOCLOADBUF, (unsigned long)buf);

	if (ret < 0) {
		warnx("error loading mixers from %s", fname);
		return 1;
	}

	return 0;
}
