/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file driver_test.cpp
 * hello_world_cxx application example for DP deftwind
 *
 * @author Example User <mail@example.com>
 */

#include <cstdio>
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/init.h>
#include <nuttx/sched.h>

#include <apps/platform/cxxinitialize.h>

#include <dp_task.h>

//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: helloxx_main
 ****************************************************************************/

extern "C"
{
static bool thread_should_exit = false;     /**< deftwind exit flag */
static bool thread_running = false;     /**< deftwind status flag */
static int deftwind_task;                /**< Handle of deftwind task / thread */

 __EXPORT int DeftWind_main(int argc, char *argv[]);

 int deftwind(int argc, char *argv[])
 {
 
     printf("[deftwind] starting\n");
 
     thread_running = true;
 
     while (!thread_should_exit) {
         printf("deftwind!\n");
         sleep(1);
     }
 
     printf("[deftwind] exiting.\n");
 
     thread_running = false;
 
     return 0;
 }

 int DeftWind_main(int argc, char *argv[])
 {
	// If C++ initialization for static constructors is supported, then do
	// that first

	up_cxxinitialize();

    deftwind_task = dp_task_create("deftwind",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     2000,
                     deftwind,
                     (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
    return 0;
}

}
