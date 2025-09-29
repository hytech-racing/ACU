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

// Public: perform one measurement and return bus voltage and current
PowerSensorData PowerSensor::read_data() {
    // 1) Raw ADC reads from configured channels
    const uint16_t raw_v   = _read_raw(_cfg.ch_voltage);      // divider node (e.g., CH0)
    const uint16_t raw_out = _read_raw(_cfg.ch_current_out);  // sensor OUT (e.g., CH1)
    const uint16_t raw_ref = _read_raw(_cfg.ch_current_ref);  // sensor REF (e.g., CH2)

    // 2) Convert ADC codes -> volts at the ADC pins
    const float v_node = (static_cast<float>(raw_v)   / _cfg.resolution) * _cfg.vref;
    const float v_out  = (static_cast<float>(raw_out) / _cfg.resolution) * _cfg.vref;
    const float v_ref  = (static_cast<float>(raw_ref) / _cfg.resolution) * _cfg.vref;

    // 3) Rebuild pack/bus voltage from divider: Vbus = Vnode * (Ra + Rb) / Rb
    const float v_bus = v_node * _cfg.voltage_divider_gain;

    // 4) Current from sensor differential: I = (Vout - Vref) / (V_per_A)
    float current = (v_out - v_ref) / _cfg.sensor_v_per_a;

    _data = PowerSensorData{ current, v_bus, current*v_bus };
    return _data;
}
