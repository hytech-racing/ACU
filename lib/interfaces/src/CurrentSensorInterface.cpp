#include "CurrentSensorInterface.h"
#include "MCPSPIInterface.h"
#include <Arduino.h> 
#include <SPI.h>
#include "SharedFirmwareTypes.h"

using mcp_spi_interface::make_cmd;
using mcp_spi_interface::read_channel_voltage;

// Initialize the ADC chip-select pin. 
void CurrentSensorInterface::init(const CurrentSensorConfig& cfg){
    _config = cfg;
    pinMode(_config.cs, OUTPUT);
    digitalWrite(_config.cs, HIGH); 
}

//Reads single values not differential
volt CurrentSensorInterface::_read_voltage(CHANNEL_CODES_e ch, bool isSingleEnded) {
    auto cmd = make_cmd(static_cast<uint8_t>(ch), isSingleEnded); 
    return read_channel_voltage(_config.cs, cmd);
}

//Checks if the data from the channel is correct
volt CurrentSensorInterface::_read_and_validate(CHANNEL_CODES_e ch, bool isSingleEnded){
    volt first_read = _read_voltage(ch, isSingleEnded);
    volt second_read = _read_voltage(ch, isSingleEnded);
    if (abs(first_read - second_read) <= _config.acceptable_error){
        return second_read;
    }
    return ERROR_CODE;
}

float CurrentSensorInterface::_calculate_current(volt voltage){
    return (voltage / _config.voltage_divider_gain) / _config.sensor_v_per_a;
}


float CurrentSensorInterface::read_current() {
    const volt voltage_current = _read_and_validate(_config.ch_current);
    const bool has_valid_current_data = (voltage_current != ERROR_CODE);
    if (has_valid_current_data){
       _current = _calculate_current(voltage_current);
    }
    return _current;
}
