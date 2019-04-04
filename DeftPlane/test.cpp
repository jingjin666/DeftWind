#include <nuttx/config.h>
#include <stdio.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>



#if 0
__BEGIN_DECLS

int __EXPORT DeftWind_main(int argc, char *argv[]);
int __EXPORT DeftWind_main(int argc, char * argv[])
{
  printf("Hello, DeftWind!\n");
  return 0;
}

__END_DECLS

#endif


const AP_HAL::HAL& hal = AP_HAL::get_HAL();


void setup(void) {
    //printf("setup-----------\r\n");
    sleep(2);

}


void loop(void)
{
    //printf("loop-----------\r\n");
    sleep(2);
}


AP_HAL_MAIN();

