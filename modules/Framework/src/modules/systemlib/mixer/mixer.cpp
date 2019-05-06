/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mixer.cpp
 *
 * Programmable multi-channel mixer library.
 */

#include <dp_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer.h"

Mixer::Mixer(ControlCallback control_cb, uintptr_t cb_handle) :
	_next(nullptr),
	_control_cb(control_cb),
	_cb_handle(cb_handle)
{
}

float
Mixer::get_control(uint8_t group, uint8_t index)
{
	float	value;

	_control_cb(_cb_handle, group, index, value);

	return value;
}


float
Mixer::scale(const mixer_scaler_s &scaler, float input)
{
	float output;

	if (input < 0.0f) {
		output = (input * scaler.negative_scale) + scaler.offset;

	} else {
		output = (input * scaler.positive_scale) + scaler.offset;
	}

	if (output > scaler.max_output) {
		output = scaler.max_output;

	} else if (output < scaler.min_output) {
		output = scaler.min_output;
	}

	return output;
}

int
Mixer::scale_check(struct mixer_scaler_s &scaler)
{
	if (scaler.offset > 1.001f) {
		return 1;
	}

	if (scaler.offset < -1.001f) {
		return 2;
	}

	if (scaler.min_output > scaler.max_output) {
		return 3;
	}

	if (scaler.min_output < -1.001f) {
		return 4;
	}

	if (scaler.max_output > 1.001f) {
		return 5;
	}

	return 0;
}

const char *
Mixer::findtag(const char *buf, unsigned &buflen, char tag)
{
	while (buflen >= 2) {
		if ((buf[0] == tag) && (buf[1] == ':')) {
			return buf;
		}

		buf++;
		buflen--;
	}

	return nullptr;
}

const char *
Mixer::skipline(const char *buf, unsigned &buflen)
{
	const char *p;

	/* if we can find a CR or NL in the buffer, skip up to it */
	if ((p = (const char *)memchr(buf, '\r', buflen)) || (p = (const char *)memchr(buf, '\n', buflen))) {
		/* skip up to it AND one beyond - could be on the NUL symbol now */
		buflen -= (p - buf) + 1;
		return p + 1;
	}

	return nullptr;
}

/****************************************************************************/

NullMixer::NullMixer() :
	Mixer(nullptr, 0)
{
}

unsigned
NullMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	if (space > 0) {
		*outputs = 0.0f;
		return 1;
	}

	return 0;
}

void
NullMixer::groups_required(uint32_t &groups)
{

}

NullMixer *
NullMixer::from_text(const char *buf, unsigned &buflen)
{
	NullMixer *nm = nullptr;

	/* enforce that the mixer ends with space or a new line */
	for (int i = buflen - 1; i >= 0; i--) {
		if (buf[i] == '\0') {
			continue;
		}

		/* require a space or newline at the end of the buffer, fail on printable chars */
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\r') {
			/* found a line ending or space, so no split symbols / numbers. good. */
			break;

		} else {
			return nm;
		}

	}

	if ((buflen >= 2) && (buf[0] == 'Z') && (buf[1] == ':')) {
		nm = new NullMixer;
		buflen -= 2;
	}

	return nm;
}
