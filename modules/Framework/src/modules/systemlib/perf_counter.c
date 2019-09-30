/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file perf_counter.c
 *
 * @brief Performance measuring tools.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/queue.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include "perf_counter.h"

#define ddeclare(...) __VA_ARGS__

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	sq_entry_t		link;	/**< list linkage */
	enum perf_counter_type	type;	/**< counter type */
	const char		*name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		event_overruns;
	uint64_t		time_start;
	uint64_t		time_total;
	uint64_t		time_least;
	uint64_t		time_most;
	float			mean;
	float			M2;
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		time_event;
	uint64_t		time_first;
	uint64_t		time_last;
	uint64_t		time_least;
	uint64_t		time_most;
	float			mean;
	float			M2;
};

/**
 * List of all known counters.
 */
static sq_queue_t	perf_counters;


perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{return NULL;}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{return NULL;}

void
perf_free(perf_counter_t handle)
{}

void
perf_count(perf_counter_t handle)
{}

void
perf_begin(perf_counter_t handle)
{}

void
perf_end(perf_counter_t handle)
{}

#include <systemlib/err.h>

void
perf_set(perf_counter_t handle, int64_t elapsed)
{}

void
perf_cancel(perf_counter_t handle)
{}



void
perf_reset(perf_counter_t handle)
{}

void
perf_print_counter(perf_counter_t handle)
{}

void
perf_print_counter_fd(int fd, perf_counter_t handle)
{}

uint64_t
perf_event_count(perf_counter_t handle)
{return 0;}

void
perf_print_all(int fd)
{}

extern const uint16_t latency_bucket_count;
extern uint32_t latency_counters[];
extern const uint16_t latency_buckets[];

void
perf_print_latency(int fd)
{}

void
perf_reset_all(void)
{}
