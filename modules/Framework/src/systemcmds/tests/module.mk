#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 60000
MAXOPTIMIZATION		 = -O0

SRCS			 =	tests_main.c \
				test_usart.c

EXTRACXXFLAGS = -Wframe-larger-than=6000
EXTRACXXFLAGS += -Wno-float-equal
EXTRACXXFLAGS += -Wno-double-promotion -Wno-error=logical-op

