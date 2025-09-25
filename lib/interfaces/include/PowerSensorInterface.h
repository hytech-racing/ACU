#ifndef POWER_SENSOR_INTERFACE_H
#define POWER_SENSOR_INTERFACE_H

#include <Arduino.h>
#include <etl/singleton.h>
#include "SharedFirmwareTypes.h" // provides `volt` typedef


using pin = size_t;

namespace pin_default_params
{
    constexpr const pin VOLTAGE_PIN = A0;
    constexpr const pin CURRENT_PIN = A1;
}

/**
 * Interface for reading pack voltage and current.
 */
class PowerSensorInterface
{
public:
    PowerSensorInterface() = delete;

    PowerSensorInterface(
        pin _voltage_pin = pin_default_params::VOLTAGE_PIN,
        pin _current_pin = pin_default_params::CURRENT_PIN)
        : _voltage_pin(_voltage_pin), _current_pin(_current_pin)
    {};

    void init(uint32_t init_millis);

    volt read_latest_voltage();

    float read_latest_current();



   

   

private:
    const pin _voltage_pin;
    const pin _current_pin;
    uint32_t init_millis_ = 0;
    const float _teensy_max_input_voltage = 3.3F;
    const float _bit_resolution = 4095.0F; // 12-bit ADC
    const float _pack_out_ts_conv_factor = 1.0f; // TODO: calibrate based on voltage divider
    const float _current_conv_factor = 1.0f; // TODO: calibrate based on voltage divider
};

using PowerSensorInstance = etl::singleton<PowerSensorInterface>;

#endif
