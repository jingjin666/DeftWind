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
#include <apps/platform/cxxinitialize.h>


//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: helloxx_main
 ****************************************************************************/

extern "C"
{
 __EXPORT int DeftWind_main(int argc, char *argv[]);

 int DeftWind_main(int argc, char *argv[])
 {
	// If C++ initialization for static constructors is supported, then do
	// that first

	up_cxxinitialize();

    while(1) {
        sleep(1);
        //printf("driver test---\n");
    }
	return 0;
  }
}
