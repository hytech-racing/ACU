#ifndef POWERSENSINGINTERFACE_H
#define POWERSENSINGINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

namespace pin_power_sensing_params
{
    constexpr const pin VOLTAGE_PIN = -1; // Confirm with hardware team
    constexpr const pin CURRENT_PIN = -1; // Confirm with hardware team
};

class PowerSensingInterface
{
public:
    PowerSensingInterface(
        pin voltage_pin = pin_power_sensing_params::VOLTAGE_PIN,
        pin current_pin = pin_power_sensing_params::CURRENT_PIN
    );

    /**
     * @brief Initializes the pins for the power sensing interface.
     * @post Pins are configured as INPUT.
    */
    void init();

    /**
     * @brief Reads the voltage from the sensor.
     * @return The measured voltage as a float.
    */
    float getVoltage();

    /**
     * @brief Reads the current from the sensor.
     * @return The measured current as a float.
    */
    float getCurrent();

private:
    const pin _voltage_pin;
    const pin _current_pin;

    // ADC and Teensy properties
    const float _bit_resolution = 4095.0F; // Teensy 4.1 12-bit ADC resolution
    const float _teensy41_max_input_voltage = 3.3F;

    // NOTE: These are placeholder conversion factors.
    // These values will depend on the chosen sensors and circuitry - TODO: Confirm with hardware team
    const float _voltage_conversion_factor = 1.0F;
    const float _current_conversion_factor = 1.0F;
};

using PowerSensingInterfaceInstance = etl::singleton<PowerSensingInterface>;

#endif // POWERSENSINGINTERFACE_H
