#include "PowerSensingInterface.h"
#include "Arduino.h"

PowerSensingInterface::PowerSensingInterface(pin voltage_pin, pin current_pin) :
    _voltage_pin(voltage_pin),
    _current_pin(current_pin)
{}

void PowerSensingInterface::init() {
    pinMode(_voltage_pin, INPUT);
    pinMode(_current_pin, INPUT);
}

float PowerSensingInterface::getVoltage() {
    // Convert to a voltage reading
    // (ADC_reading * V_ref / ADC_resolution) / conversion_factor
    // The conversion factor depends on the sensor and any voltage dividers - TODO: Confirm with hardware team
    int adc_reading = analogRead(_voltage_pin);
    float voltage = static_cast<float>(adc_reading) * (_teensy41_max_input_voltage / _bit_resolution) / _voltage_conversion_factor;
    return voltage;
}

float PowerSensingInterface::getCurrent() {
    int adc_reading = analogRead(_current_pin);
    float adc_voltage = static_cast<float>(adc_reading) * (_teensy41_max_input_voltage / _bit_resolution);
    // TODO: Clarify this with POC: Currentlya placeholder for the actual current calculation.
    float current = adc_voltage / _current_conversion_factor;
    return current;
}
