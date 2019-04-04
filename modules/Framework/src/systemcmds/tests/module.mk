#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 60000
MAXOPTIMIZATION		 = -O0

SRCS			 =	tests_main.c \
					test_usdhc.c \
					test_usart.c \
					test_led.c \
					test_i2c.cpp

EXTRACXXFLAGS = -Wframe-larger-than=6000
EXTRACXXFLAGS += -Wno-float-equal

# Flag is only valid for GCC, not clang
ifneq ($(USE_GCC), 0)
EXTRACXXFLAGS += -Wno-double-promotion -Wno-error=logical-op
endif