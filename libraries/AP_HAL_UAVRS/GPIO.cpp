#include <board_config.h>
#include <platforms/dp_micro_hal.h>
#include "GPIO.h"
#include <stdio.h>
#include <drivers/drv_gpio.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


using namespace UAVRS;

#define LOW     0
#define HIGH    1

extern const AP_HAL::HAL& hal;


GPIO::GPIO()
{}

void GPIO::init()
{
	_gpio_fmu_fd = open(FMU_DEVICE_PATH, 0);
    if (_gpio_fmu_fd == -1) {
        AP_HAL::panic("Unable to open GPIO");
    }
}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    uint32_t pinmask;
	uint8_t old_value;

    switch (pin) {
    case UAVRS_GPIO_FMU_SERVO_PIN(0) ... UAVRS_GPIO_FMU_SERVO_PIN(5):
        pinmask = 1U<<(pin-UAVRS_GPIO_FMU_SERVO_PIN(0));
        if (output) {
            old_value = read(pin);
            if (old_value) {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_HIGH, pinmask);
            } else {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_LOW, pinmask);
            }
        } else {
            ioctl(_gpio_fmu_fd, GPIO_SET_INPUT, pinmask);
        }
        break;
	
#ifdef GPIO_CAMERA_TRIGGER
	case UAVRS_GPIO_CAMER_TRRIGER_RELAY_PIN:
        pinmask = GPIO_CAMERA_TRIGGER;
        if (output) {
            old_value = read(pin);
            if (old_value) {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_HIGH, pinmask);
            } else {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_LOW, pinmask);
            }
        } else {
            ioctl(_gpio_fmu_fd, GPIO_SET_INPUT, pinmask);
        }
        break;
#endif
        }
}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
	switch (pin) {
	    case UAVRS_GPIO_FMU_SERVO_PIN(0) ... UAVRS_GPIO_FMU_SERVO_PIN(5):
	        // the only pins that can be mapped are the FMU servo rail pins */
	        return pin;
#ifdef GPIO_CAMERA_FEEDBACK
		case UAVRS_GPIO_CAMER_FEEDBACK_INPUT_PIN:
			return pin;
#endif
    }

    return -1;
}

uint8_t GPIO::read(uint8_t pin) {
    switch (pin) {
    case UAVRS_GPIO_FMU_SERVO_PIN(0) ... UAVRS_GPIO_FMU_SERVO_PIN(5): {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & (1U<<(pin-UAVRS_GPIO_FMU_SERVO_PIN(0))))?HIGH:LOW;
        }
	
#ifdef GPIO_CAMERA_TRIGGER
	case UAVRS_GPIO_CAMER_TRRIGER_RELAY_PIN: {
			uint32_t relays = 0;
			ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
			return (relays & GPIO_CAMERA_TRIGGER)?HIGH:LOW;
		}
#endif

#ifdef GPIO_CAMERA_FEEDBACK
	case UAVRS_GPIO_CAMER_FEEDBACK_INPUT_PIN: {
			uint32_t relays = 0;
			ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
			return (relays & GPIO_CAMERA_FEEDBACK)?HIGH:LOW;
		}
#endif
    }
    return LOW;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    switch (pin) {
	    case UAVRS_GPIO_FMU_SERVO_PIN(0) ... UAVRS_GPIO_FMU_SERVO_PIN(5):
	        ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, 1U<<(pin-UAVRS_GPIO_FMU_SERVO_PIN(0)));
	        break;

#ifdef GPIO_CAMERA_TRIGGER
		case UAVRS_GPIO_CAMER_TRRIGER_RELAY_PIN:
	        ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_CAMERA_TRIGGER);
			break;
#endif

#ifdef GPIO_CAMERA_FEEDBACK
		case UAVRS_GPIO_CAMER_FEEDBACK_INPUT_PIN:
			ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_CAMERA_FEEDBACK);
			break;
#endif
    }
}

void GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(0);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GPIO::usb_connected(void)
{
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1) || defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    return dp_arch_gpioread(GPIO_USB_OTG_VBUS) && _usb_connected;
#else
    return false;
#endif
}

bool GPIO::imu_data_ready(void)
{
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1) || defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    return dp_arch_gpioread(GPIO_ADIS_DRDY);
#else
    return false;
#endif
}

void GPIO::imu_reset(bool level)
{
#if defined(CONFIG_ARCH_BOARD_UAVRS_V1) || defined(CONFIG_ARCH_BOARD_UAVRS_V2)
    dp_arch_gpiowrite(GPIO_ADIS_RESET, level);
#endif
}

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read() {
    return _v;
}

void DigitalSource::write(uint8_t value) {
    _v = value;
}

void DigitalSource::toggle() {
    _v = !_v;
}
