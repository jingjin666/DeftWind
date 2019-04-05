/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_log.cpp
 * 
 */

#include <stdlib.h>
#include <dp_log.h>
#ifdef __DP_POSIX
#include <execinfo.h>
#endif

__EXPORT int __dp_log_level_current = DP_LOG_LEVEL_AT_RUN_TIME;

__EXPORT const char *__dp_log_level_str[_DP_LOG_LEVEL_PANIC + 1] = { "INFO", "DEBUG", "WARN", "ERROR", "PANIC" };

void dp_backtrace()
{
#ifdef __DP_POSIX
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = backtrace(buffer, 10);
	callstack = backtrace_symbols(buffer, bt_size);

	DP_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		DP_INFO("%s", callstack[idx]);
	}

	free(callstack);
#endif
}
