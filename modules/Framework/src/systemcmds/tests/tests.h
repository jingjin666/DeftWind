/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

#ifndef __DP_SYSTEMCMDS_TESTS_H
#define __DP_SYSTEMCMDS_TESTS_H

/**
 * @file tests.h
 * Tests declaration file.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dp_config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__BEGIN_DECLS

extern int test_usdhc(int argc, char *argv[]);
extern int test_usart(int argc, char *argv[]);
extern int test_led(int argc, char *argv[]);
extern int test_i2c(int argc, char *argv[]);
extern int test_spi(int argc, char *argv[]);
extern int test_servo(int argc, char *argv[]);
__END_DECLS

#endif /* __DP_SYSTEMCMDS_TESTS_H */
