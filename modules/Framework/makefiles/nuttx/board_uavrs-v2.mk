#
# Board-specific definitions for the UAVRSv2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM7F
CONFIG_BOARD			 = UAVRS_V2

include $(DP_MK_DIR)toolchain_gnu-arm-eabi.mk
