/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_task.h
 * Preserve existing task API call signature with OS abstraction
 */

#pragma once

#include <stdbool.h>

#ifdef __DP_ROS

#error "DP tasks not supported in ROS"

#elif defined(__DP_NUTTX)
typedef int dp_task_t;

#include <sys/prctl.h>
#define dp_prctl prctl

/** Default scheduler type */
#if CONFIG_RR_INTERVAL > 0
# define SCHED_DEFAULT  SCHED_RR
#else
# define SCHED_DEFAULT  SCHED_FIFO
#endif

#define dp_task_exit(x) _exit(x)

#elif defined(__DP_POSIX) || defined(__DP_QURT)
#include <pthread.h>
#include <sched.h>

/** Default scheduler type */
#define SCHED_DEFAULT	SCHED_FIFO
#ifdef __DP_LINUX
#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT (((sched_get_priority_max(SCHED_FIFO) - sched_get_priority_min(SCHED_FIFO)) / 2) + sched_get_priority_min(SCHED_FIFO))
#elif defined(__DP_DARWIN)
#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT (((sched_get_priority_max(SCHED_FIFO) - sched_get_priority_min(SCHED_FIFO)) / 2) + sched_get_priority_min(SCHED_FIFO))
#elif defined(__DP_QURT)
#define SCHED_PRIORITY_MAX 255
#define SCHED_PRIORITY_MIN 0
#define SCHED_PRIORITY_DEFAULT 20
#else
#error "No target OS defined"
#endif

#if defined (__DP_LINUX)
#include <sys/prctl.h>
#else
#define PR_SET_NAME	1
#endif

typedef int dp_task_t;

typedef struct {
	int argc;
	char **argv;
} dp_task_args_t;
#else
#error "No target OS defined"
#endif

typedef int (*dp_main_t)(int argc, char *argv[]);

__BEGIN_DECLS

/** Reboots the board */
__EXPORT void dp_systemreset(bool to_bootloader) noreturn_function;

/** Starts a task and performs any specific accounting, scheduler setup, etc. */
__EXPORT dp_task_t dp_task_create(const char *name,
				       int priority,
				       int scheduler,
				       int stack_size,
				       dp_main_t entry,
				       char *const argv[]);

/** Deletes a task - does not do resource cleanup **/
__EXPORT int dp_task_delete(dp_task_t pid);

/** Send a signal to a task **/
__EXPORT int dp_task_kill(dp_task_t pid, int sig);

/** Exit current task with return value **/
__EXPORT void dp_task_exit(int ret);

/** Show a list of running tasks **/
__EXPORT void dp_show_tasks(void);

/** See if a task is running **/
__EXPORT bool dp_task_is_running(const char *taskname);

#ifdef __DP_POSIX
/** set process (and thread) options */
__EXPORT int dp_prctl(int option, const char *arg2, unsigned pid);
#endif

__END_DECLS

