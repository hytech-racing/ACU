#include "PowerSensor.h"
#include "MCPSPIInterface.h"
#include <Arduino.h> 
#include <SPI.h>
#include "SharedFirmwareTypes.h"

using mcp_spi_interface::make_cmd;
using mcp_spi_interface::read_channel_voltage;

// Initialize the ADC chip-select pin.
void PowerSensor::init() {
    pinMode(_cfg.cs, OUTPUT);
    digitalWrite(_cfg.cs, HIGH); 
}

//Reads single values not differential
volt PowerSensor::_read_voltage(CHANNEL_CODES_e ch) {
    auto cmd = make_cmd(static_cast<uint8_t>(ch), true); 
    return read_channel_voltage(_cfg.cs, cmd);
}

//Checks if the data from the channel is correct
volt PowerSensor::_read_and_validate(CHANNEL_CODES_e ch){
    uint16_t first_read = _read_voltage(ch);
    uint16_t second_read = _read_voltage(ch);
    if (abs(first_read - second_read) <= _cfg.acceptable_error){
        return second_read;
    }
    return ERROR_CODE;
}

float PowerSensor::_calculate_current(uint16_t v_out, uint16_t v_ref){
    return (v_out - v_ref) / _cfg.sensor_v_per_a;
}

volt PowerSensor::_calculate_voltage(uint16_t voltage){
    return voltage*_cfg.voltage_divider_gain;
}

float PowerSensor::_calculate_power(){
    return _data.current * _data.power;
}


// Public: perform one measurement and return bus voltage and current
PowerSensorData PowerSensor::read_data() {
    
    const uint16_t voltage_ts = _read_and_validate(_cfg.ch_voltage);      // divider node (e.g., CH0)
    const uint16_t voltage_out = _read_and_validate(_cfg.ch_current_out);  // sensor OUT (e.g., CH1)
    const uint16_t voltage_ref = _read_and_validate(_cfg.ch_current_ref);  // sensor REF (e.g., CH2)

    const bool has_valid_current_data = (voltage_ref != ERROR_CODE && voltage_out != ERROR_CODE);
    const bool has_valid_voltage_data = (voltage_ts != ERROR_CODE);
    if (has_valid_current_data){
       _data.current = _calculate_current(voltage_out, voltage_ref);
    }
    if (has_valid_voltage_data){
        _data.voltage =  _calculate_voltage(voltage_ts);
    }
    if (has_valid_current_data && has_valid_voltage_data){
        _data.power = _calculate_power();
    }
    return _data;
}
