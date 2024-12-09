#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

/* CONSTANT Definitions */
/**
 * These devices have 2 IDLE modes: 
 * REFUP - the references remain on meaning getting to the MEASURE state is almost instantaneous
 * STANDY - the references are offline, so it takes longer to get to MEASURE state
 * For our applications we don't need the references to remain on, neither do we need the cell data
 * instantaneously. When we configure the device we will need to set a value for REFUP
 * Because we want to use the STANDBY state, we will set DEVICE_REFUP_MODE to false / 0b0
 * If we ever want tot use the REFUP state, change this to true / 0b1
 * The following MACROS are for writing the configuration:
*/ 
const bool device_refup_mode = false;
const bool adcopt = false;
const uint16_t gpios_enabled = 0x1F; // There are 5 GPIOs, we are using all 5 so they are all given a 1
const bool dcto_read = 0x1;
const bool dcto_write = 0x0;
const int total_ic = 12;      // Number of LTC6811-2 ICs that are used in the accumulator
const int even_ic_cells = 12; // Number of cells monitored by ICs with even addresses
const int odd_ic_cells = 9;   // Number of cells monitored by ICS with odd addresses
const int chip_select_9 = 9;
const int chip_select_10 = 10;
const int adc_conversion_cell_select_mode = 0;
const int adc_conversion_gpio_select_mode = 0;
const uint8_t discharge_permitted = 0x0;
const uint8_t adc_mode_cv_conversion = 0x1;
const uint8_t adc_mode_gpio_conversion = 0x11;
const int minimum_voltage = 30000;   // Minimum allowable single cell voltage in units of 100μV
const int maximum_voltage = 42000;   // Maxiumum allowable single cell voltage in units of 100μV
const int maximum_total_voltage = 5330000;    // Maximum allowable pack total voltage in units of 100μV    
const int maximum_thermistor_voltage = 26225; // Maximum allowable pack temperature corresponding to 60C in units 100μV
const uint16_t under_voltage_threshold = 1874;  // 3.0V  // Minimum voltage value following datasheet formula: Comparison Voltage = (VUV + 1) • 16 • 100μV
const uint16_t over_voltage_threshold = 2625;   // 4.2V  // Maximum voltage value following datasheet formula: Comparison Voltage = VOV • 16 • 100μV
const uint16_t gpio_enable = 0x1F;
const uint16_t CRC15_POLY = 0x4599;             // Used for calculating the PEC table for LTC6811
const int cv_adc_conversion_time_us = 13;
const int gpio_adc_conversion_time_us = 3.1;


/* ENUM Definitions */
// Command Codes 
enum class CMD_CODES_e {
    // WRITES
    WRITE_CONFIG = 0x1,
    WRITE_S_CONTROL = 0x14,
    WRITE_PWM = 0x20,
    WRITE_COMM = 0x721,
    // READS
    READ_CONFIG = 0x2,
    READ_CELL_VOLTAGE_GROUP_A = 0x4,
    READ_CELL_VOLTAGE_GROUP_B = 0x6,
    READ_CELL_VOLTAGE_GROUP_C = 0x8,
    READ_CELL_VOLTAGE_GROUP_D = 0xA,
    READ_GPIO_VOLTAGE_GROUP_A = 0xC,
    READ_GPIO_VOLTAGE_GROUP_B = 0xE,
    READ_STATUS_GROUP_A = 0x10,
    READ_STATUS_GROUP_B = 0x12,  
    READ_S_CONTROL = 0x16,
    READ_PWM = 0x22,
    READ_COMM = 0x722,
    // STARTS
    START_S_CONTROL = 0x19,
    START_CV_ADC_CONVERSION = 0x260,
    START_GPIO_ADC_CONVERSION = 0x460,
    START_CV_GPIO_ADC_CONVERSION = 0x46F,
    START_CV_SC_CONVERSION = 0x467,
    START_COMM = 0x723,
    // CLEARS
    CLEAR_S_CONTROL = 0x18,
    CLEAR_GPIOS = 0x712,
    CLEAR_STATUS = 0x713,
    // POLL ADC STATUS, DIAGNOSE MUX
    POLL_ADC_STATUS = 0x714,
    DIAGNOSE_MUX_POLL_STATUS = 0x715
}; 

enum class ADC_MODE_e : uint8_t {
	MODE_ZERO = 0x0,
	FAST = 0x1,
	NORMAL = 0x2,
	FILTERED = 0x3
};

#endif