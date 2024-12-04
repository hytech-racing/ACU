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

/* Instances of LTC6811 (2) */
BMSDriverGroup<12, 2> BMSGroup;

void setup()
{
    pinMode(6, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(6, HIGH); // write Teensy_OK pin high

    BMSGroup.init();

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
    //BMSData data_in = BMSGroup.read_data();

    // Perform Calculations for cell balancing

    // Report Voltage & temps over to Ethernet

    // Parse EM Messages from CAN to Ethernet

}