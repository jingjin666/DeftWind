/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file hello_world.c
 * hello_world application example for DP deftwind
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dp_config.h>
#include <nuttx/sched.h>

//#include <systemlib/systemlib.h>
//#include <systemlib/err.h>
#include <dp_task.h>
#define warnx printf

static bool thread_should_exit = false;		/**< hello_world exit flag */
static bool thread_running = false;		/**< hello_world status flag */
static int hello_world_task;				/**< Handle of hello_world task / thread */

/**
 * hello_world management function.
 */
__EXPORT int hello_world_main(int argc, char *argv[]);

/**
 * Mainloop of hello_world.
 */
static int hello_world(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: hello_world {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The hello_world app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int hello_world_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("hello_world already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		hello_world_task = dp_task_create("hello_wold",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 hello_world,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int hello_world(int argc, char *argv[])
{

	warnx("[hello_world] starting\n");

	thread_running = true;

	while (!thread_should_exit) {
		warnx("Hello World!\n");
		sleep(10);
	}

	warnx("[hello_world] exiting.\n");

	thread_running = false;

	return 0;
}
