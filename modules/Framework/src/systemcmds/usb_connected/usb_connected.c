/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file usb_connected.c
 *
 * utility to check if USB is connected. Used in startup scripts
 *
 * @author Andrew Tridgell
 */

#include <dp_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <arch/board/board.h>
#include <board_config.h>

__EXPORT int usb_connected_main(int argc, char *argv[]);

int usb_connected_main(int argc, char *argv[])
{
	return dp_arch_gpioread(GPIO_USB_OTG_VBUS);
}
