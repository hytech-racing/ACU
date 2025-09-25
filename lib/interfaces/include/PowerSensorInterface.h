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
        uint32_t init_millis,
        pin voltage_pin = pin_default_params::VOLTAGE_PIN,
        pin current_pin = pin_default_params::CURRENT_PIN)
        : voltage_pin_(voltage_pin), current_pin_(current_pin)
    {};

    void init(uint32_t init_millis);

    volt read_latest_voltage(uint32_t curr_millis);

    float read_latest_current(uint32_t curr_millis);



   

   

private:
    const pin voltage_pin_;
    const pin current_pin_;
    uint32_t init_millis_ = 0;
    static constexpr float TEENSY_MAX_INPUT_VOLTAGE = 3.3F;
    static constexpr float BIT_RESOLUTION = 4095.0F; // 12-bit ADC
    static constexpr float PACK_AND_TS_OUT_CONV_FACTOR = 0.00482F; // TODO: calibrate
    static constexpr float CURRENT_CONV_FACTOR = 0.5118F; // TODO: calibrate
};

using PowerSensorInstance = etl::singleton<PowerSensorInterface>;

#endif
