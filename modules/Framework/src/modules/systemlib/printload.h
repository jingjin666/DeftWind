/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file printload.h
 *
 * Print the current system load.
 *
 * @author Lorenz Meier <lorenz@dp.io>
 */

#pragma once

#include <dp_config.h>

#include <stdint.h>

#ifndef CONFIG_MAX_TASKS
#define CONFIG_MAX_TASKS 64
#endif

struct print_load_s {
	uint64_t total_user_time;

	int running_count;
	int blocked_count;

	uint64_t new_time;
	uint64_t interval_start_time;
	uint32_t last_times[CONFIG_MAX_TASKS]; // in [ms]. This wraps if a process needs more than 49 days of CPU
	float interval_time_ms_inv;
};

__BEGIN_DECLS

__EXPORT void init_print_load_s(uint64_t t, struct print_load_s *s);

__EXPORT void print_load(uint64_t t, int fd, struct print_load_s *print_state);


typedef void (*print_load_callback_f)(void *user);

/**
 * Print load to a buffer, and call cb after each written line (buffer will not include '\n')
 */
void print_load_buffer(uint64_t t, char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state);

__END_DECLS
