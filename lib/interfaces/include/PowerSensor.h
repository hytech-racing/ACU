#ifndef POWERSENSOR_H
#define POWERSENSOR_H
#include <SharedFirmwareTypes.h>
#include <stdint.h>

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
struct PowerSensorData
{
    float current; // Amps
    volt  voltage; // Volts (pack/bus)
    float power; // Power
};

// Constants you actually need (set once at construction)
struct Config {
    float acceptable_error;
    int cs;                         // MCP3208 chip-select pin
    float vref;                     // ADC reference (V) – MCP3208 VREF pin
    float resolution;   
    float voltage_divider_gain;
    float sensor_v_per_a;           // V per A for your current sensor (e.g., 0.006667f)
    CHANNEL_CODES_e ch_voltage      = CHANNEL_CODES_e::CH0; // divider node channel
    CHANNEL_CODES_e ch_current_out  = CHANNEL_CODES_e::CH1; // sensor OUT channel
    CHANNEL_CODES_e ch_current_ref  = CHANNEL_CODES_e::CH2; // sensor REF channel
};

/**
 * Minimal power sensor wrapped around one MCP3208 (U20 in your schematic).
 * - Voltage from a divider on CH (default CH0)
 * - Current from sensor OUT and REF on CH1 and CH2 (OUT - REF) / (V_per_A)
 */
class PowerSensor
{
public:

    PowerSensor() = delete;
    PowerSensor(const Config& cfg) : _cfg(cfg) {};

    void init();
    // One-shot read: returns bus voltage and current.
    PowerSensorData read_data();

private:
    uint16_t _read_raw(CHANNEL_CODES_e ch);

    bool _is_valid(CHANNEL_CODES_e ch, uint16_t raw);

private:
    Config _cfg;
    PowerSensorData _data;
};

#endif // POWERSENSOR_H
