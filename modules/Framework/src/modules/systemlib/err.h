/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file err.h
 *
 * Simple error/warning functions, heavily inspired by the BSD functions of
 * the same names.
 *
 * The err() and warn() family of functions display a formatted error
 * message on the standard error output.  In all cases, the last
 * component of the program name, a colon character, and a space are
 * output.  If the fmt argument is not NULL, the printf(3)-like formatted
 * error message is output.  The output is terminated by a newline
 * character.
 *
 * The err(), errc(), verr(), verrc(), warn(), warnc(), vwarn(), and
 * vwarnc() functions append an error message obtained from strerror(3)
 * based on a supplied error code value or the global variable errno,
 * preceded by another colon and space unless the fmt argument is NULL.
 *
 * In the case of the errc(), verrc(), warnc(), and vwarnc() functions,
 * the code argument is used to look up the error message.
 *
 * The err(), verr(), warn(), and vwarn() functions use the global
 * variable errno to look up the error message.
 *
 * The errx() and warnx() functions do not append an error message.
 *
 * The err(), verr(), errc(), verrc(), errx(), and verrx() functions do
 * not return, but exit with the value of the argument eval.
 *
 */

#ifndef _SYSTEMLIB_ERR_H
#define _SYSTEMLIB_ERR_H

#include <dp_log.h>
#include <stdarg.h>
#include "visibility.h"

__BEGIN_DECLS

__EXPORT const char *getprogname(void);

#ifdef __DP_NUTTX
#define EXIT(eval) exit(eval)
#else
#define EXIT(eval) dp_task_exit(eval)
#endif

#define err(eval, ...)		do { \
		DP_ERR(__VA_ARGS__); \
		DP_ERR("Task exited with errno=%i\n", errno); \
		EXIT(eval); \
	} while(0)

#define errx(eval, ...)		do { \
		DP_ERR(__VA_ARGS__); \
		EXIT(eval); \
	} while(0)

#define warn(...) 		DP_WARN(__VA_ARGS__)
#define warnx(...) 		DP_WARN(__VA_ARGS__)

__END_DECLS

#endif
