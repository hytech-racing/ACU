#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H
#include <SharedFirmwareTypes.h>
#include <stdint.h>
#include "etl/singleton.h"
#include "MCPSPIInterface.h"

struct CurrentSensorConfig {
    float acceptable_error = 0.02f; // Volts
    int cs = 0;                         // MCP3208 chip-select pin
    float voltage_divider_gain = 1.0f;
    float sensor_v_per_a = 0.005f;           // V per A for your current sensor (e.g., 0.005 or 5mV/A)
    CHANNEL_CODES_e ch_current        = CHANNEL_CODES_e::CH3; // channel to read current from
};

/**
 * Minimal Current sensor wrapped around one MCP3208 
 */
class CurrentSensorInterface
{
public:
    static constexpr volt ERROR_CODE = -1; 

    void init(const CurrentSensorConfig& cfg);

    // One-shot read: returns and validates bus voltage and current.
    float read_current();

private:
    volt _read_voltage(CHANNEL_CODES_e ch, bool isSingleEnded = true);
    
    float _calculate_current(volt voltage);
    volt _read_and_validate(CHANNEL_CODES_e ch, bool isSingleEnded = true);

private:
    CurrentSensorConfig _config;
    float _current;
    
};
using CurrentSensorInstance = etl::singleton<CurrentSensorInterface>;
#endif // CurrentSENSOR_H
