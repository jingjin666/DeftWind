#
# Board-specific startup code for the UAVRSv2
#

SRCS		 = imxrt_boot.c \
		   imxrt_flexspi_nor_boot.c \
		   imxrt_flexspi_nor_flash.c \
		   imxrt_bringup.c \
		   imxrt_appinit.c \
		   imxrt_autoleds.c \
		   uavrs2_led.c
		   

MAXOPTIMIZATION	 = -Os
