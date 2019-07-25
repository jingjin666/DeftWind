#
# Board-specific startup code for the UAVRSv2
#

SRCS		 = uavrs_v2_init.c \
			 uavrs_v2_usdhc.c \
		   uavrs_v2_flexspi_nor_boot.c \
		   uavrs_v2_flexspi_nor_flash.c \
		   uavrs_v2_led.c \
		   uavrs_v2_spi.c \
		   uavrs_v2_usb.c \
		   uavrs_v2_timer_config.c \
           uavrs_v2_sdram_ini_dcd.c

MAXOPTIMIZATION	 = -Os
