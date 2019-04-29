#include "AnalogIn.h"

using namespace UAVRS;

AnalogSource::AnalogSource(float v) :
    _v(v)
{}

float AnalogSource::read_average() {
    return _v;
}

float AnalogSource::voltage_average() {
    return 5.0f * _v / 1024.0f;
}

float AnalogSource::voltage_latest() {
    return 5.0f * _v / 1024.0f;
}

float AnalogSource::read_latest() {
    return _v;
}

void AnalogSource::set_pin(uint8_t p)
{}

void AnalogSource::set_stop_pin(uint8_t p)
{}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn::AnalogIn()
{}

void AnalogIn::init()
{}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {
    return new AnalogSource(1.11);
}

float AnalogIn::board_voltage(void)
{
    return 5.0f;
}

void AnalogIn::_timer_tick(void)
{
}

