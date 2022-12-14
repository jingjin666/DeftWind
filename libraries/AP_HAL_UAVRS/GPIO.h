#pragma once

#include "AP_HAL_UAVRS.h"

#define UAVRS_GPIO_PIEZO_PIN              110
#define UAVRS_GPIO_EXT_FMU_RELAY1_PIN     111
#define UAVRS_GPIO_EXT_FMU_RELAY2_PIN     112
#define UAVRS_GPIO_EXT_IO_RELAY1_PIN      113
#define UAVRS_GPIO_EXT_IO_RELAY2_PIN      114
#define UAVRS_GPIO_EXT_IO_ACC1_PIN        115
#define UAVRS_GPIO_EXT_IO_ACC2_PIN        116
#define UAVRS_GPIO_CAMER_TRRIGER_RELAY_PIN 117
#define UAVRS_GPIO_CAMER_FEEDBACK_INPUT_PIN 118


/*
  start servo channels used as GPIO at 50. Pin 50 is
  the first FMU servo pin
 */
#define UAVRS_GPIO_FMU_SERVO_PIN(n)       (n+50)

#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif


class UAVRS::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);

    /* return true if imu data ready */
	bool imu_data_ready(void);

	/* reset imu by hardware io */
	void imu_reset(bool);

    void set_usb_connected(void) { _usb_connected = true; }

private:
    int _gpio_fmu_fd = -1;
    int _gpio_io_fd = -1;
    bool _usb_connected = false;
};

class UAVRS::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    uint8_t _v;
};
