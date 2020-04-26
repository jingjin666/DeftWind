/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file led.cpp
 *
 * LED driver.
 */

#include <dp_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_led.h>
#include <stdio.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#ifdef __DP_NUTTX
class LED : device::CDev
#else
class LED : device::VDev
#endif
{
public:
	LED();
	virtual ~LED();

	virtual int		init();
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);
};

LED::LED() :
#ifdef __DP_NUTTX
	CDev("led", LED0_DEVICE_PATH)
#else
	VDev("led", LED0_DEVICE_PATH)
#endif
{
	// force immediate init/device registration
	init();
}

LED::~LED()
{
}

int
LED::init()
{
#ifdef __DP_NUTTX
	CDev::init();
#else
	VDev::init();
#endif
	led_init();

	return 0;
}

int
LED::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case LED_ON:
		led_on(arg);
		break;

	case LED_OFF:
		led_off(arg);
		break;

	case LED_TOGGLE:
		led_toggle(arg);
		break;


	default:
#ifdef __DP_NUTTX
		result = CDev::ioctl(filp, cmd, arg);
#else
		result = VDev::ioctl(filp, cmd, arg);
#endif
	}

	return result;
}

namespace
{
LED	*gLED;
}

void
drv_led_start(void)
{
	if (gLED == nullptr) {
		gLED = new LED;

		if (gLED != nullptr) {
			gLED->init();
		}
	}
}
