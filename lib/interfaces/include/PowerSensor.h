#ifndef POWERSENSOR_H
#define POWERSENSOR_H
#include <SharedFirmwareTypes.h>
#include <stdint.h>


struct PowerSensorData
{
    float current; // Amps
    volt  voltage; // Volts (pack/bus)
    float power; // Power
};

// The codes for each channel on the MCP3028 (D2 D1 D0)
enum class CHANNEL_CODES_e : uint8_t
{
    CH0 = 0,
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH6 = 6,
    CH7 = 7,
};

struct Config {
    int acceptable_error;
    int cs;                         // MCP3208 chip-select pin
    float voltage_divider_gain;
    float sensor_v_per_a = 0.005;           // V per A for your current sensor (e.g., 0.005 or 5mV/A)
    CHANNEL_CODES_e ch_voltage      = CHANNEL_CODES_e::CH0; // divider node channel
    // CHANNEL_CODES_e ch_current_out  = CHANNEL_CODES_e::CH1; // sensor OUT channel
    // CHANNEL_CODES_e ch_current_ref  = CHANNEL_CODES_e::CH2; // sensor REF channel
    CHANNEL_CODES_e ch_current        = CHANNEL_CODES_e::CH3; // channel to read current from
};

/**
 * Minimal power sensor wrapped around one MCP3208 
 */
class PowerSensor
{
public:
    static constexpr volt ERROR_CODE = -1; 

    PowerSensor() = delete;
    PowerSensor(const Config& cfg) : _cfg(cfg) {};

    void init();

    // One-shot read: returns and validates bus voltage and current.
    PowerSensorData read_data();

private:
    volt _read_voltage(CHANNEL_CODES_e ch, bool isSingleEnded = true);
    
    float _calculate_current(uint16_t voltage);
    volt _calculate_voltage(uint16_t v_raw);
    float _calculate_power();
    volt _read_and_validate(CHANNEL_CODES_e ch, bool isSingleEnded = true);

private:
    Config _cfg;
    PowerSensorData _data;
    
};

#endif // POWERSENSOR_H
