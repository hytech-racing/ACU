/* -------------------- System Includes -------------------- */
#include "PowerSensorInterface.h"


/* -------------------- Project Includes -------------------- */
#include "../../../../include/Configuration.h"


// Note: This implementation uses blocking analogRead(). For higher sample rates
// replace with a non-blocking Teensy ADC API (or DMA-based ADC) and buffer the
// results. This file intentionally omits documentation headers per project
// guidelines; public documentation lives in the header file.


void PowerSensorInterface::init(uint32_t init_millis)
{
    init_millis_ = init_millis;

    // Configure analog pins as inputs (explicit; analogRead works without this
    // on many cores but being explicit clarifies intent).
    pinMode(voltage_pin_, INPUT);
    pinMode(current_pin_, INPUT);
}


volt PowerSensorInterface::read_latest_voltage(uint32_t /* curr_millis */)
{
    const float raw = static_cast<float>(analogRead(voltage_pin_));
    const float volts = raw * (TEENSY_MAX_INPUT_VOLTAGE / BIT_RESOLUTION) * PACK_AND_TS_OUT_CONV_FACTOR;
    return static_cast<volt>(volts);
}


float PowerSensorInterface::read_latest_current(uint32_t /* curr_millis */)
{
    const float raw = static_cast<float>(analogRead(current_pin_));
    return raw * (TEENSY_MAX_INPUT_VOLTAGE / BIT_RESOLUTION) * CURRENT_CONV_FACTOR;
}
