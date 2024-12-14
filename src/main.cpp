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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> energy_meter_can;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> telem_can;


/* Instances of LTC6811 (2) */
BMSDriverGroup<12, 2> BMSGroup;

void setup()
{
    pinMode(teensy_OK_pin, OUTPUT);
    pinMode(teensy_to_vehicle_watchdog_pin, OUTPUT); 
    digitalWrite(6, HIGH); // write Teensy_OK pin high

    BMSGroup.init();

    Serial.begin(115200);
    SPI.begin();
    telem_can.begin();
    telem_can.setBaudRate(500000);
    energy_meter_can.begin();
    energy_meter_can.setBaudRate(500000);
    energy_meter_can.enableMBInterrupts();
    //energy_meter_can.onReceive(parse_energy_meter_can_message);
    for (int i = 0; i < 64; i++)
    {                                                                          // Fill all filter slots with Charger Control Unit message filter
        telem_can.setMBFilter(static_cast<FLEXCAN_MAILBOX>(i), CCU_STATUS_CANID); // Set CAN mailbox filtering to only watch for charger controller status CAN messages
    }
    analogReadResolution(12);
}

void loop()
{
    telem_can.events();
    energy_meter_can.events();

    // READ IC data
    auto data = BMSGroup.read_data();

    // Perform Calculations for cell balancing

    // Report Voltage & temps over to Ethernet

    // Parse EM Messages from CAN to Ethernet

}