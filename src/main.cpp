/**
 * PREAMBLE: MUST READ TO UNDERSTAND WTF IS GOING ON
 * 
*/

/* Library Includes */
#include <Arduino.h>
#include "LTC6811.h"
#include "ACUCalculations.h"
#include "DataContainer.h"
#include "MessageInterface.h"
#include "hytech.h"

// Might not need this
uint16_t max_humidity = 0;
uint16_t max_thermistor_voltage = 0;
uint16_t min_thermistor_voltage = 65535;
uint16_t max_board_temp_voltage = 0;
uint16_t min_board_temp_voltage = 65535;

/* AMS CAN messages */

BMS_STATUS_t bms_status_;
BMS_TEMPS_t bms_temperatures_;
ACU_SHUNT_MEASUREMENTS_t acu_shunt_measurements_;
BMS_VOLTAGES_t bms_voltages_;
BMS_DETAILED_VOLTAGES_t bms_detailed_voltages_;
BMS_DETAILED_TEMPS_t bms_detailed_temperatures_;
EM_MEASUREMENT_t em_measurements_;

/* Locations of min and max voltage, temps, and humidity */
IC_Cell_Location_s min_voltage_location;
IC_Cell_Location_s max_voltage_location;
IC_Cell_Location_s max_board_temp_location;
IC_Cell_Location_s min_board_temp_location;
IC_Cell_Location_s max_thermistor_location;
IC_Cell_Location_s max_humidity_location;
IC_Cell_Location_s min_thermistor_location;

/* Fault Trackers */
Fault_State_s over_voltage;
Fault_State_s under_voltage;
Fault_State_s over_temperature;
Fault_State_s pack_over_voltage;

/* Timer Tracker */
Elapsed_Timers_s timers;


void setup()
{
}

void loop()
{
}