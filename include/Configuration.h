#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <cstdint>
#include <stddef.h>
#include <array>

/* CONSTANT Definitions */
/**
 * These devices have 2 IDLE modes: 
 * REFUP - the references remain on meaning getting to the MEASURE state is almost instantaneous
 * STANDBY - the references are offline, so it takes longer to get to MEASURE state
 * For our applications we don't need the references to remain on, neither do we need the cell data
 * instantaneously. When we configure the device we will need to set a value for REFUP
 * Because we want to use the STANDBY state, we will set DEVICE_REFUP_MODE to false / 0b0
 * If we ever want tot use the REFUP state, change this to true / 0b1
 * The following MACROS are for writing the configuration:
*/ 
const bool device_refup_mode = true;
const bool adcopt = false;
const uint16_t gpios_enabled = 0x1F; // There are 5 GPIOs, we are using all 5 so they are all given a 1
const bool dcto_read = 0x1;
const bool dcto_write = 0x0;
const int adc_conversion_cell_select_mode = 0;
const int adc_conversion_gpio_select_mode = 0;
const uint8_t discharge_permitted = 0x0;
const uint8_t adc_mode_cv_conversion = 0x1;
const uint8_t adc_mode_gpio_conversion = 0x1;
const uint16_t under_voltage_threshold = 1874;  // 3.0V  // Minimum voltage value following datasheet formula: Comparison Voltage = (VUV + 1) • 16 • 100μV
const uint16_t over_voltage_threshold = 2625;   // 4.2V  // Maximum voltage value following datasheet formula: Comparison Voltage = VOV • 16 • 100μV
const uint16_t gpio_enable = 0x1F;
const uint16_t CRC15_POLY = 0x4599;             // Used for calculating the PEC table for LTC6811
const float cv_adc_conversion_time_us = 13.0f;
const float gpio_adc_conversion_time_us = 3.1f;

#endif