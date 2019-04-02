/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tests_main.c
 * Tests main file, loads individual tests.
 *
 * @author User <mail@example.com>
 */

#include <dp_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>
//#include <systemlib/perf_counter.h>

#include "tests.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int test_help(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct {
	const char 	*name;
	int	(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
#define OPT_NOJIGTEST	(1<<2)
} tests[] = {
    {"usdhc",       test_usdhc, OPT_NOJIGTEST | OPT_NOALLTEST},
	{"usart",		test_usart,	OPT_NOJIGTEST | OPT_NOALLTEST},
	{"help",		test_help,	OPT_NOALLTEST | OPT_NOHELP | OPT_NOJIGTEST},
	{NULL,			NULL, 		0}
};

#define NTESTS (sizeof(tests) / sizeof(tests[0]))

static int
test_help(int argc, char *argv[])
{
	unsigned	i;

	printf("Available tests:\n");

	for (i = 0; tests[i].name; i++) {
		printf("  %s\n", tests[i].name);
	}

	return 0;
}
__EXPORT int tests_main(int argc, char *argv[]);

/**
 * Executes system tests.
 */
int tests_main(int argc, char *argv[])
{
	unsigned	i;

	if (argc < 2) {
		printf("tests: missing test name - 'tests help' for a list of tests\n");
		return 1;
	}

	for (i = 0; tests[i].name; i++) {
		if (!strcmp(tests[i].name, argv[1])) {
			return tests[i].fn(argc - 1, argv + 1);
		}
	}

	printf("tests: no test called '%s' - 'tests help' for a list of tests\n", argv[1]);
	return ERROR;
}
