/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_log.h
 * Platform dependant logging/debug implementation
 */

#pragma once

#define _DP_LOG_LEVEL_ALWAYS	0
#define _DP_LOG_LEVEL_DEBUG		1
#define _DP_LOG_LEVEL_WARN		2
#define _DP_LOG_LEVEL_ERROR		3
#define _DP_LOG_LEVEL_PANIC		4

// Used to silence unused variable warning
static inline void do_nothing(int level, ...)
{
	(void)level;
}

/****************************************************************************
 * __dp_log_omit:
 * Compile out the message
 ****************************************************************************/
#define __dp_log_omit(level, FMT, ...)   do_nothing(level, ##__VA_ARGS__)

#include <inttypes.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <dp_defines.h>
#if 0

__BEGIN_DECLS
__EXPORT extern uint64_t hrt_absolute_time(void);

__EXPORT extern const char *__dp_log_level_str[5];
__EXPORT extern int __dp_log_level_current;
__EXPORT extern void dp_backtrace(void);
__END_DECLS

#define DP_BACKTRACE() dp_backtrace()

// __dp_log_level_current will be initialized to DP_LOG_LEVEL_AT_RUN_TIME
#define DP_LOG_LEVEL_AT_RUN_TIME	_DP_LOG_LEVEL_ERROR

/****************************************************************************
 * Implementation of log section formatting based on printf
 *
 * To write to a specific stream for each message type, open the streams and
 * set __dp__log_startline to something like:
 * 	if (level <= __dp_log_level_current) printf(_dp_fd[level],
 *
 * Additional behavior can be added using "{\" for __dp__log_startline and
 * "}" for __dp__log_endline and any other required setup or teardown steps
 ****************************************************************************/
#define __dp__log_printcond(cond, ...)	    if (cond) printf(__VA_ARGS__)
#define __dp__log_printline(level, ...)    if (level <= __dp_log_level_current) printf(__VA_ARGS__)

#define __dp__log_timestamp_fmt	"%-10" PRIu64 " "
#define __dp__log_timestamp_arg 	,hrt_absolute_time()
#define __dp__log_level_fmt		"%-5s "
#define __dp__log_level_arg(level)	,__dp_log_level_str[level]
#define __dp__log_thread_fmt		"%#X "
#define __dp__log_thread_arg		,(unsigned int)pthread_self()

#define __dp__log_file_and_line_fmt 	" (file %s line %u)"
#define __dp__log_file_and_line_arg 	, __FILE__, __LINE__
#define __dp__log_end_fmt 		"\n"
#define __dp__log_endline 		)

/****************************************************************************
 * Output format macros
 * Use these to implement the code level macros below
 ****************************************************************************/


/****************************************************************************
 * __dp_log_named_cond:
 * Convert a message in the form:
 * 	DP_LOG_COND(__dbg_enabled, "val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", "LOG", val);
 * if the first arg/condition is true.
 ****************************************************************************/
#define __dp_log_named_cond(name, cond, FMT, ...) \
	__dp__log_printcond(cond,\
			     "%s " \
			     FMT\
			     __dp__log_end_fmt \
			     ,name, ##__VA_ARGS__\
			    )

/****************************************************************************
 * __dp_log:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", __dp_log_level_str[3], val);
 ****************************************************************************/
#define __dp_log(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt \
			     FMT\
			     __dp__log_end_fmt \
			     __dp__log_level_arg(level), ##__VA_ARGS__\
			    )

/****************************************************************************
 * __dp_log_timestamp:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu val is %d\n", __dp_log_level_str[3],
 *		hrt_absolute_time(), val);
 ****************************************************************************/
#define __dp_log_timestamp(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_timestamp_fmt\
			     FMT\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_timestamp_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __dp_log_timestamp_thread:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu %#X val is %d\n", __dp_log_level_str[3],
 *		hrt_absolute_time(), pthread_self(), val);
 ****************************************************************************/
#define __dp_log_timestamp_thread(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_timestamp_fmt\
			     __dp__log_thread_fmt\
			     FMT\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_timestamp_arg\
			     __dp__log_thread_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __dp_log_file_and_line:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d (file %s line %u)\n",
 *		__dp_log_level_str[3], val, __FILE__, __LINE__);
 ****************************************************************************/
#define __dp_log_file_and_line(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_timestamp_fmt\
			     FMT\
			     __dp__log_file_and_line_fmt\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __dp__log_file_and_line_arg\
			    )

/****************************************************************************
 * __dp_log_timestamp_file_and_line:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu val is %d (file %s line %u)\n",
 *		__dp_log_level_str[3], hrt_absolute_time(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __dp_log_timestamp_file_and_line(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_timestamp_fmt\
			     FMT\
			     __dp__log_file_and_line_fmt\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __dp__log_file_and_line_arg\
			    )

/****************************************************************************
 * __dp_log_thread_file_and_line:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s %#X val is %d (file %s line %u)\n",
 *		__dp_log_level_str[3], pthread_self(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __dp_log_thread_file_and_line(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_thread_fmt\
			     FMT\
			     __dp__log_file_and_line_fmt\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_thread_arg\
			     , ##__VA_ARGS__\
			     __dp__log_file_and_line_arg\
			    )

/****************************************************************************
 * __dp_log_timestamp_thread_file_and_line:
 * Convert a message in the form:
 * 	DP_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu %#X val is %d (file %s line %u)\n",
 *		__dp_log_level_str[3], hrt_absolute_time(),
 *		pthread_self(), val, __FILE__, __LINE__);
 ****************************************************************************/
#define __dp_log_timestamp_thread_file_and_line(level, FMT, ...) \
	__dp__log_printline(level,\
			     __dp__log_level_fmt\
			     __dp__log_timestamp_fmt\
			     __dp__log_thread_fmt\
			     FMT\
			     __dp__log_file_and_line_fmt\
			     __dp__log_end_fmt\
			     __dp__log_level_arg(level)\
			     __dp__log_timestamp_arg\
			     __dp__log_thread_arg\
			     , ##__VA_ARGS__\
			     __dp__log_file_and_line_arg\
			    )


/****************************************************************************
 * Code level macros
 * These are the log APIs that should be used by the code
 ****************************************************************************/

/****************************************************************************
 * Messages that should never be filtered or compiled out
 ****************************************************************************/
#define DP_LOG(FMT, ...) 	__dp_log(_DP_LOG_LEVEL_ALWAYS, FMT, ##__VA_ARGS__)
#define DP_INFO(FMT, ...) 	__dp_log(_DP_LOG_LEVEL_ALWAYS, FMT, ##__VA_ARGS__)

#if defined(TRACE_BUILD)
/****************************************************************************
 * Extremely Verbose settings for a Trace build
 ****************************************************************************/
#define DP_PANIC(FMT, ...)	__dp_log_timestamp_thread_file_and_line(_DP_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define DP_ERR(FMT, ...)	__dp_log_timestamp_thread_file_and_line(_DP_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define DP_WARN(FMT, ...) 	__dp_log_timestamp_thread_file_and_line(_DP_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define DP_DEBUG(FMT, ...) 	__dp_log_timestamp_thread(_DP_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(DEBUG_BUILD)
/****************************************************************************
 * Verbose settings for a Debug build
 ****************************************************************************/
#define DP_PANIC(FMT, ...)	__dp_log_timestamp_file_and_line(_DP_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define DP_ERR(FMT, ...)	__dp_log_timestamp_file_and_line(_DP_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define DP_WARN(FMT, ...) 	__dp_log_timestamp_file_and_line(_DP_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define DP_DEBUG(FMT, ...) 	__dp_log_timestamp(_DP_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(RELEASE_BUILD)
/****************************************************************************
 * Non-verbose settings for a Release build to minimize strings in build
 ****************************************************************************/
#define DP_PANIC(FMT, ...)	__dp_log(_DP_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define DP_ERR(FMT, ...)	__dp_log(_DP_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define DP_WARN(FMT, ...) 	__dp_log_omit(_DP_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define DP_DEBUG(FMT, ...) 	__dp_log_omit(_DP_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#else
/****************************************************************************
 * Medium verbose settings for a default build
 ****************************************************************************/
#define DP_PANIC(FMT, ...)	__dp_log(_DP_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define DP_ERR(FMT, ...)	__dp_log(_DP_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define DP_WARN(FMT, ...) 	__dp_log(_DP_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define DP_DEBUG(FMT, ...) 	__dp_log_omit(_DP_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#endif
#define DP_LOG_NAMED(name, FMT, ...) 	__dp_log_named_cond(name, true, FMT, ##__VA_ARGS__)
#define DP_LOG_NAMED_COND(name, cond, FMT, ...) __dp_log_named_cond(name, cond, FMT, ##__VA_ARGS__)
#else
#define DP_PANIC printf
#define DP_ERR printf
#define DP_WARN printf
#define DP_DEBUG printf
#define DP_LOG printf
#define DP_INFO printf
#define DP_LOG_NAMED(name, FMT, ...) 
#define DP_LOG_NAMED_COND(name, cond, FMT, ...)
#endif
