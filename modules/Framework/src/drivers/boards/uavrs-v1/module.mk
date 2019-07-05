#
# Board-specific startup code for the UAVRSv1
#

SRCS		 = uavrs_v1_usb.c \
		   uavrs_v1_init.c \
		   uavrs_v1_led.c \
		   uavrs_v1_spi.c \
		   uavrs_v1_can.c \
		   uavrs_v1_timer_config.c \
		   uavrs_v1_reset.c

MAXOPTIMIZATION	 = -Os
