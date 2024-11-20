#ifndef __DATA_CONTAINER_H__
#define __DATA_CONTAINER_H__

/* Library Includes */
#include <string.h>
#include <stdint.h>
#include "FlexCAN_T4.h"
#include "HyTech_CAN.h"
#include "hytech.h"

/* MACRO Definitions */
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
#define DEVICE_REFUP_MODE false
#define ADCOPT false
#define GPIOS_ENABLED 0x1F            // There are 5 GPIOs, we are using all 5 so they are all given a 1
#define DCTO_READ 0x1
#define DCTO_WRITE 0x0
// These are generic MACROS
#define TOTAL_IC 12                  // Number of LTC6811-2 ICs that are used in the accumulator
#define EVEN_IC_CELLS 12             // Number of cells monitored by ICs with even addresses
#define ODD_IC_CELLS 9               // Number of cells monitored by ICS with odd addresses
#define CHIP_SELECT_GROUP_ONE 9      // Chip select for first LTC6820 corresponding to first group of cells
#define CHIP_SELECT_GROUP_TWO 10     // Chip select for second LTC6820 corresponding to second group of cells
#define ADC_CONVERSION_CELL_SELECT_MODE 0
#define ADC_CONVERSION_GPIO_SELECT 0
#define DISCHARGE_PERMITTED 0x0
#define ADC_MODE_CV_CONVERSION 0x1
#define ADC_MODE_GPIO_CONVERSION 0x11

// Change to USING_LTC6811-#, where # is 1 or 2 depending on what's being used on the car
#define USING_LTC6811_2

// DON'T EDIT THIS IF ELSE
#ifdef USING_LTC6811_1
    const int data_in_count = 48;  // 48 because each IC provides 8 bytes of data each
#else
    const int data_in_count = 8;
#endif

/* CONSTANT Definitions */
const uint16_t under_voltage_threshold = 1874;  // 3.0V  // Minimum voltage value following datasheet formula: Comparison Voltage = (VUV + 1) • 16 • 100μV
const uint16_t over_voltage_threshold = 2625;   // 4.2V  // Maximum voltage value following datasheet formula: Comparison Voltage = VOV • 16 • 100μV
const uint16_t gpio_enable = 0x1F;
const uint16_t CRC15_POLY = 0x4599;             // Used for calculating the PEC table for LTC6811
const int cv_adc_conversion_time_ms = 13;
const int gpio_adc_conversion_time_ms = 3.1;

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


/* STRUCT Definitions */
struct Fault_State_s {
    bool is_flagged;
    int fault_counter;
};

struct IC_Cell_Location_s {
    int icIndex;
    int cellIndex;
};

struct IC_Voltage_Temp_Data_s {
    // There are 12 cell voltages per IC we CAN record. For some, we might only use 9
    uint16_t cell_voltage[12];
    // For each IC, we want to record if we decide to balance / discharge it
    bool balance_status[12];
    // There are 6 gpio voltages per IC we want to record
    uint16_t gpio_voltage[6];
    // Thus, there are 6 floating point temperatures   
    float gpio_temperatures[6];
};

struct IC_Buffers_s {
    uint16_t cell_voltage_A[3];
    uint16_t cell_voltage_B[3];
    uint16_t cell_voltage_C[3];
    uint16_t cell_voltage_D[3];
    uint16_t gpio_voltage_A[3];
    uint16_t gpio_voltage_B[3];
    uint8_t config_byte[6];
};

struct IC_Config_s {
    int chip_select;
#ifdef USING_LTC6811_2
    int address; 
#endif
};

struct Elapsed_Timers_s {
    elapsedMillis can_bms_status_timer = 0;
    elapsedMillis can_bms_detailed_voltages_timer = 2;
    elapsedMillis can_bms_detailed_temps_timer = 4;
    elapsedMillis can_bms_voltages_timer = 6;
    elapsedMillis can_bms_temps_timer = 8;
    elapsedMillis can_bms_onboard_temps_timer = 10;
    elapsedMicros CC_integrator_timer = 0; // Timer used to provide estimation of pack charge from shunt current
};

#endif