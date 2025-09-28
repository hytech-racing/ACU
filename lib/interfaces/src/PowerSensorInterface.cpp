#include "PowerSensorInterface.h"


#include "../../../../include/Configuration.h"

void PowerSensorInterface::init(uint32_t init_millis)
{
    init_millis_ = init_millis;
    pinMode(_voltage_pin, INPUT);
    pinMode(_current_pin, INPUT);
}


volt PowerSensorInterface::read_latest_voltage()
{
    float raw = static_cast<float>(analogRead(_voltage_pin));
    Serial.print("Raw Voltage: ");
    Serial.println(raw, 3);
    return static_cast<volt>(raw * (_teensy_max_input_voltage / _bit_resolution) * _pack_out_ts_conv_factor);
    // return 1.0f;
}


float PowerSensorInterface::read_latest_current()
{

    float raw = static_cast<float>(analogRead(_current_pin));
    Serial.print("Raw Current: ");
    Serial.println(raw, 3);
    return raw * (_teensy_max_input_voltage / _bit_resolution) * _current_conv_factor;
    // return 1.0f;
}
