/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file conversions.c
 * Implementation of commonly used conversions.
 */

#include <dp_config.h>
#include <float.h>

#include "conversions.h"

int16_t
int16_t_from_bytes(uint8_t bytes[])
{
	union {
		uint8_t    b[2];
		int16_t    w;
	} u;

	u.b[1] = bytes[0];
	u.b[0] = bytes[1];

	return u.w;
}
