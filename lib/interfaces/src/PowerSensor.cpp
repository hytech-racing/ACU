#include "PowerSensor.h"
#include "MCPSPIInterface.h"
#include <Arduino.h> 
#include <SPI.h>

using mcp_spi_interface::make_cmd;
using mcp_spi_interface::read_channel;

// Initialize the ADC chip-select pin.
void PowerSensor::init() {
    pinMode(_cfg.cs, OUTPUT);
    digitalWrite(_cfg.cs, HIGH); 
}

//Reads single values not differential
uint16_t PowerSensor::_read_raw(CHANNEL_CODES_e ch) {
    auto cmd = make_cmd(static_cast<uint8_t>(ch), true);
    return read_channel(_cfg.cs, cmd);
}

bool PowerSensor::_is_valid(CHANNEL_CODES_e ch, uint16_t raw){
    if (abs(_read_raw(ch) - raw) <= _cfg.acceptable_error){
        return true;
    }
    return false;
}
// Public: perform one measurement and return bus voltage and current
PowerSensorData PowerSensor::read_data() {
    // 1) Raw ADC reads from configured channels
    const uint16_t raw_v   = _read_raw(_cfg.ch_voltage);      // divider node (e.g., CH0)
    bool raw_v_is_valid = _is_valid(_cfg.ch_voltage, raw_v);
    const uint16_t raw_out = _read_raw(_cfg.ch_current_out);  // sensor OUT (e.g., CH1)
    bool raw_out_is_valid = _is_valid(_cfg.ch_current_out, raw_out);
    const uint16_t raw_ref = _read_raw(_cfg.ch_current_ref);  // sensor REF (e.g., CH2)
    bool raw_ref_is_valid = _is_valid(_cfg.ch_current_ref, raw_ref);
    if (raw_out_is_valid && raw_ref_is_valid){
        const float v_out  = (static_cast<float>(raw_out) / _cfg.resolution) * _cfg.vref;
        const float v_ref  = (static_cast<float>(raw_ref) / _cfg.resolution) * _cfg.vref;
        _data.current = (v_out - v_ref) / _cfg.sensor_v_per_a;
    }
    if (raw_v_is_valid){
        const float v_node = (static_cast<float>(raw_v)   / _cfg.resolution) * _cfg.vref;
        _data.voltage =  v_node * _cfg.voltage_divider_gain;
    }
    if (raw_ref_is_valid && raw_out_is_valid && raw_v_is_valid){
        _data.power = _data.current * _data.voltage;
    }
    return _data;
}
