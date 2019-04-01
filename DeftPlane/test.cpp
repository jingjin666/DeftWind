#include <nuttx/config.h>
#include <stdio.h>

__BEGIN_DECLS

int __EXPORT DeftWind_main(int argc, char *argv[]);
int __EXPORT DeftWind_main(int argc, char * argv[])
{
  printf("Hello, DeftWind!\n");
  return 0;
}

__END_DECLS