/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mixer_load.c
 *
 * Programmable multi-channel mixer library.
 */

#include <dp_config.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer_load.h"

int load_mixer_file(const char *fname, char *buf, unsigned maxlen)
{
	FILE		*fp;
	char		line[120];

	/* open the mixer definition file */
	fp = fopen(fname, "r");

	if (fp == NULL) {
		warnx("file not found");
		return -1;
	}

	/* read valid lines from the file into a buffer */
	buf[0] = '\0';

	for (;;) {

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {
			break;
		}

		/* if the line doesn't look like a mixer definition line, skip it */
		if ((strlen(line) < 2) || !isupper(line[0]) || (line[1] != ':')) {
			continue;
		}

		/* compact whitespace in the buffer */
		char *t, *f;

		for (f = line; *f != '\0'; f++) {
			/* scan for space characters */
			if (*f == ' ') {
				/* look for additional spaces */
				t = f + 1;

				while (*t == ' ') {
					t++;
				}

				if (*t == '\0') {
					/* strip trailing whitespace */
					*f = '\0';

				} else if (t > (f + 1)) {
					memmove(f + 1, t, strlen(t) + 1);
				}
			}
		}

		/* if the line is too long to fit in the buffer, bail */
		if ((strlen(line) + strlen(buf) + 1) >= maxlen) {
			warnx("line too long");
			fclose(fp);
			return -1;
		}

		/* add the line to the buffer */
		strcat(buf, line);
	}

	fclose(fp);
	return 0;
}

