/**
 * PREAMBLE: MUST READ TO UNDERSTAND WTF IS GOING ON
 */

/* Library Includes */
#include <Arduino.h>
#include "BMSDriverGroup.h"
#include "ACUCalculations.h"
#include "Configuration.h"
#include "MessageInterface.h"
#include "hytech.h"

// Might not need this
uint16_t max_humidity = 0;
uint16_t max_thermistor_voltage = 0;
uint16_t min_thermistor_voltage = 65535;
uint16_t max_board_temp_voltage = 0;
uint16_t min_board_temp_voltage = 65535;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> ENERGY_METER_CAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> TELEM_CAN;

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

/* Instances of LTC6811 (2) */
BMSDriverGroup BMSGroup_CS9;  // Holds all of the data and functionality for LTCs on chip select 9
BMSDriverGroup BMSGroup_CS10; // Holds all of the data and functionality for LTCs on chip select 10

void setup()
{
    pinMode(6, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(6, HIGH); // write Teensy_OK pin high

    BMSGroup_CS9 = BMSDriverGroup(chip_select_9, 6);
    BMSGroup_CS10 = BMSDriverGroup(chip_select_10, 6);
    BMSGroup_CS9.init();
    BMSGroup_CS10.init();
    int address_cs9[6] = {0, 1, 6, 7, 8, 9};
    int address_cs10[6] = {2, 3, 4, 5, 10, 11};
    BMSGroup_CS9.set_address(address_cs9);
    BMSGroup_CS10.set_address(address_cs10);

    Serial.begin(115200);
    SPI.begin();

    TELEM_CAN.begin();
    TELEM_CAN.setBaudRate(500000);
    ENERGY_METER_CAN.begin();
    ENERGY_METER_CAN.setBaudRate(500000);
    ENERGY_METER_CAN.enableMBInterrupts();
    //ENERGY_METER_CAN.onReceive(parse_energy_meter_can_message);
    for (int i = 0; i < 64; i++)
    {                                                                          // Fill all filter slots with Charger Control Unit message filter
        TELEM_CAN.setMBFilter(static_cast<FLEXCAN_MAILBOX>(i), CCU_STATUS_CANID); // Set CAN mailbox filtering to only watch for charger controller status CAN messages
    }
    analogReadResolution(12);
}

void loop()
{
    TELEM_CAN.events();
    ENERGY_METER_CAN.events();

    // READ IC data
    BMSGroup_CS9.read_voltages();
    BMSGroup_CS10.read_voltages();
    BMSGroup_CS9.read_GPIOs();
    BMSGroup_CS10.read_GPIOs();

    // Perform Calculations for cell balancing

    // Report Voltage & temps over to Ethernet

    // Parse EM Messages from CAN to Ethernet

}