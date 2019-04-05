/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file usb.c
 *
 * Board-specific USB functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <dp_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <up_arch.h>
#include <chip.h>
//#include <imxrt_usbotg.h>
#include "board_config.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  imxrt_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide imxrt_usbpullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following EXTERN can be redefined to be
 *   NULL.
 *
 ************************************************************************************/
int imxrt_usbpullup(FAR struct usbdev_s *dev, bool enable);
__EXPORT
int imxrt_usbpullup(FAR struct usbdev_s *dev, bool enable)
{
	usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

	return OK;
}

/************************************************************************************
 * Name:  imxrt_usbsuspend
 *
 * Description:
 *   Board logic must provide the imxrt_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

void imxrt_usbsuspend(FAR struct usbdev_s *dev, bool resume);
__EXPORT
void imxrt_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}

/************************************************************************************
 * Name: board_read_VBUS_state
 *
 * Description:
 *   All boards must provide a way to read the state of VBUS, this my be simple
 *   digital input on a GPIO. Or something more complicated like a Analong input
 *   or reading a bit from a USB controller register.
 *
 * Returns -  0 if connected.
 *
 ************************************************************************************/
int board_read_VBUS_state(void);
int board_read_VBUS_state(void)
{
	return 0; //todo:Add USB controller register
}

void up_usbinitialize(void);
__EXPORT 
void up_usbinitialize(void)
{
	uinfo("initialize: %d");
} 