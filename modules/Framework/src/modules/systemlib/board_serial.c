/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file board_serial.h
 * Read off the board serial
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#include "otp.h"
#include "board_config.h"
#include "board_serial.h"

int get_board_serial(uint8_t *serialid)
{
#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
	const volatile uint32_t *udid_ptr = (const uint32_t *)UDID_START;
	union udid id;
	val_read((uint32_t *)&id, udid_ptr, sizeof(id));


	/* Copy the serial from the chips non-write memory and swap endianess */
	serialid[0] = id.data[3];   serialid[1] = id.data[2];  serialid[2] = id.data[1];  serialid[3] = id.data[0];
	serialid[4] = id.data[7];   serialid[5] = id.data[6];  serialid[6] = id.data[5];  serialid[7] = id.data[4];
	serialid[8] = id.data[11];   serialid[9] = id.data[10];  serialid[10] = id.data[9];  serialid[11] = id.data[8];
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    
#endif
	return 0;
}
