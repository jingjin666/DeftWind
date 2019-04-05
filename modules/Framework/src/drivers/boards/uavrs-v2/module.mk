#
# Board-specific startup code for the UAVRSv2
#

SRCS		 = uavrs_v2_init.c \
			 uavrs_v2_sdhc.c \
		   uavrs_v2_flexspi_nor_boot.c \
		   uavrs_v2_flexspi_nor_flash.c \
		   uavrs_v2_led.c \
		   uavrs_v2_spi.c
		   

MAXOPTIMIZATION	 = -Os
