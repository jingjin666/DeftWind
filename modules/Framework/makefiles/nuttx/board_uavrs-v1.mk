#
# Board-specific definitions for the UAVRSv1
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = UAVRS_V1

include $(DP_MK_DIR)toolchain_gnu-arm-eabi.mk
