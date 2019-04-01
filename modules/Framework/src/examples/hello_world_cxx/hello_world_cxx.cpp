/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file hello_world_cxx.cpp
 * hello_world_cxx application example for DP deftwind
 *
 * @author Example User <mail@example.com>
 */

#include <cstdio>
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/init.h>
#include <apps/platform/cxxinitialize.h>

class E {
public:
	E() {
		printf("\nE---\n");
	}
	virtual void hello(void) const = 0;
};

class C : public E {
public:
	C() {
		printf("\nC---\n");
	}
	void putString(void);
	virtual void hello(void) const override;
};
void C::putString(void)
{
	printf("\n----------------------================C\n");
}
void C::hello(void) const
{
	printf("hello-------------------------------------\n");
}

C c;
E &c_temp = c;

//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: helloxx_main
 ****************************************************************************/

extern "C"
{
 __EXPORT int hello_world_cxx_main(int argc, char *argv[]);

 int hello_world_cxx_main(int argc, char *argv[])
 {
	// If C++ initialization for static constructors is supported, then do
	// that first

	up_cxxinitialize();
	c_temp.hello();
	return 0;
  }
}
