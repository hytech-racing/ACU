/**
 * PREAMBLE: MUST READ TO UNDERSTAND WTF IS GOING ON
 */

/* Library Includes */
#include <Arduino.h>
#include "BMSDriverGroup.h"
#include "ACUController.h"
#include "Configuration.h"
#include "ACUEthernetInterface.h"
#include "hytech.h"

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> energy_meter_can;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> telem_can;

/* Instances of LTC6811 (2) */
using chip_type = LTC6811_Type_e;

// Initialize chip_select, chip_select_per_chip, and address
const int num_chips = 12;
const int num_chip_selects = 2;
std::array<int, num_chip_selects> cs = {9, 10};
std::array<int, num_chips> cs_per_chip = {9, 9, 10, 10, 10, 10, 9, 9, 9, 9, 10, 10};
std::array<int, num_chips> addr = {0,1,2,3,4,5,6,7,8,9,10,11};
ACU_State_s<num_chips> acu_state = {};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);


void setup()
{
    pinMode(teensy_OK_pin, OUTPUT);
    pinMode(teensy_to_vehicle_watchdog_pin, OUTPUT); 
    digitalWrite(6, HIGH); // write Teensy_OK pin high

    BMSGroup.init();

    Serial.begin(115200);
    SPI.begin();
    //telem_can.begin();
    //telem_can.setBaudRate(500000);
    //energy_meter_can.begin();
    //energy_meter_can.setBaudRate(500000);
    //energy_meter_can.enableMBInterrupts();
    //energy_meter_can.onReceive(parse_energy_meter_can_message);
    for (int i = 0; i < 64; i++)
    {                                                                          // Fill all filter slots with Charger Control Unit message filter
        // telem_can.setMBFilter(static_cast<FLEXCAN_MAILBOX>(i), CCU_STATUS_CANID); // Set CAN mailbox filtering to only watch for charger controller status CAN messages
    }
    analogReadResolution(12);
}

void loop()
{
    // READ IC data
    auto data = BMSGroup.read_data();

    // Perform Calculations for cell balancing

    // Report Voltage & temps over to Ethernet

    // Parse EM Messages from CAN to Ethernet

}