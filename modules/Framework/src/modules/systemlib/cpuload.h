/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#ifdef CONFIG_SCHED_INSTRUMENTATION

#include <sched.h>
#include <stdint.h>
#include <stdbool.h>

struct system_load_taskinfo_s {
	uint64_t total_runtime;			///< Runtime since start (start_time - total_runtime)/(start_time - current_time) = load
	uint64_t curr_start_time;		///< Start time of the current scheduling slot
#ifdef __DP_NUTTX
	FAR struct tcb_s *tcb;			///<
#endif
	bool valid;						///< Task is currently active / valid
};

struct system_load_s {
	uint64_t start_time;			///< Global start time of measurements
	struct system_load_taskinfo_s tasks[CONFIG_MAX_TASKS];
	uint8_t initialized;
	int total_count;
	int running_count;
	int sleeping_count;
};

__BEGIN_DECLS

__EXPORT extern struct system_load_s system_load;

__EXPORT void cpuload_initialize_once(void);

__END_DECLS

#endif
