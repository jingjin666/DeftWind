#pragma once

#define HAL_BOARD_NAME "UAVRS"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/UAVRS/LOGS"
#define HAL_BOARD_RAW_DATA_DIRECTORY "/fs/microsd/UAVRS/RAW_DATA"
#define HAL_BOARD_POS_DATA_DIRECTORY "/fs/microsd/UAVRS/POS_DATA"
#define HAL_INS_DEFAULT HAL_INS_UAVRS
#define HAL_BARO_DEFAULT HAL_BARO_UAVRS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_UAVRS

#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_UAVRS_V1
#define HAL_STORAGE_SIZE            16384
//#define HAL_WITH_UAVCAN             1
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_UAVRS_V2
#define HAL_STORAGE_SIZE            16384
#define HAL_WITH_UAVCAN				1
#else
#error "Unknown UAVRS board type"
#endif

/* 此处暂时关闭安全开关选项
 ** 打开会让UAVRSUtil::safety_switch_state 处理uorb异常
 ** 导致hal->Schedler->delay() 执行失败，直接进入shell
 */
#define HAL_HAVE_SAFETY_SWITCH 0

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_ADIS16XXX_NAME "adis16375"

#define HAL_BARO_MS5611_NAME "ms5611"
