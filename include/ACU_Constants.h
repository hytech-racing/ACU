#ifndef ACU_CONSTANTS
#define ACU_CONSTANTS

/* Standard Library */
#include <stddef.h>
#include <stdio.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <cstddef>
#include <iostream>
#include <array>
#include <algorithm>

using pin = uint8_t;
using time_us = uint32_t;


namespace ACUInterfaces
{
    /* General Interface Constants */
    const uint8_t ANALOG_READ_RESOLUTION = 12;
    const uint32_t SERIAL_BAUDRATE = 115200;

    constexpr pin ADC0_NOT_SHDN_PIN = 9;
    constexpr pin ADC0_CS_PIN = 10;
    constexpr pin ADC0_MOSI_PIN = 11;
    constexpr pin ADC0_MISO_PIN = 12;
    constexpr pin ADC0_CLK_PIN = 13;
    constexpr uint32_t ADC0_SPEED = 1000000; // 1 MHz

    /* ADC Versions*/
    /* Channels on ADC */
    constexpr int ISO_PACK_N_CHANNEL         = 0;
    constexpr int ISO_PACK_P_CHANNEL         = 1;
    constexpr int PACK_VOLTAGE_SENSE_CHANNEL = 2;
    constexpr int SHUNT_CURRENT_OUT_CHANNEL  = 3;
    constexpr int SHUNT_CURRENT_P_CHANNEL    = 4;
    constexpr int SHUNT_CURRENT_N_CHANNEL    = 5;
    constexpr int TS_OUT_FILTERED_CHANNEL    = 6;
    constexpr int PACK_OUT_FILTERED_CHANNEL  = 7;

    /* SCALE/OFFSETS on ADC */
    const float ISO_PACK_N_SCALE = 0.0656553030302;
    const float ISO_PACK_P_SCALE = 0.0656553030302;
    const float PACK_VOLTAGE_SENSE_SCALE = 0.0410345643939;
    const float SHUNT_CURRENT_OUT_SCALE = 0.03125;
    const float SHUNT_CURRENT_P_SCALE = 0.00025;
    const float SHUNT_CURRENT_N_SCALE = 0.00025;
    const float TS_OUT_FILTERED_SCALE = 0.0547254764211;
    const float PACK_OUT_FILTERED_SCALE = 0.0547254764211;
    const float ISO_PACK_N_OFFSET = 0;
    const float ISO_PACK_P_OFFSET = 0;
    const float PACK_VOLTAGE_SENSE_OFFSET = 0;
    const float SHUNT_CURRENT_OUT_OFFSET = -250;
    const float SHUNT_CURRENT_P_OFFSET = 0;
    const float SHUNT_CURRENT_N_OFFSET = 0;
    const float TS_OUT_FILTERED_OFFSET = 0;
    const float PACK_OUT_FILTERED_OFFSET = 0;

    constexpr const size_t TEENSY_OK_PIN = 3; // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    constexpr const size_t WD_KICK_PIN = 4;       // > Needs to flip at 100 Hz to keep BMS_OK high
    constexpr const size_t SW_NOT_OK_PIN = 5;  // should be HIGH by default, and then set LOW after traversing state machine
    constexpr const size_t N_FAULTED_STATE_PIN = 6;    // > Input to Safety Light, true when teensy is not in FAULT state

    constexpr const size_t BSPD_CURRENT_PIN = 15;
    constexpr const size_t SHDN_OUT_PIN = 16; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    constexpr const size_t PRECHARGE_PIN = 17; // READ from PRECHARGE
    constexpr const size_t HV_PLUS_OUT_OK_PIN = 19; // READ from HV OUT OK
    constexpr const size_t MAIN_OK_PIN = 20;
    constexpr const size_t MAIN_UNDER_THRESH_PIN = 21;
    constexpr const size_t PRECHARGE_THRESH_PIN = 22;
    constexpr const size_t IMD_OK_PIN = 23; // < READ from IMD hardware, go to FAULT state if HIGH
    constexpr const size_t PACK_OUT_FILTERED_PIN = 24;
    constexpr const size_t TS_OUT_FILTERED_PIN = 25;

    constexpr const size_t SPI1_MOSI_PIN = 26;
    constexpr const size_t SPI1_SCK_PIN = 27;
    constexpr const size_t SPI1_MISO_PIN = 39;

    constexpr const size_t SCALED_24V_PIN = 41;

    constexpr const float SHUTDOWN_CONV_FACTOR = 0.1155F; // voltage divider -> 4.7k / (4.7k + 36k)
    constexpr const float PRECHARGE_CONV_FACTOR = 0.6623F; // voltage divider -> 10k / (5.1k + 10k)
    constexpr const float PACK_AND_TS_OUT_CONV_FACTOR = 0.00482F;
    constexpr const float SHDN_OUT_CONV_FACTOR = 0.11545F;
    constexpr const float BSPD_CURRENT_CONV_FACTOR = 0.5118F;
    constexpr const float GLV_CONV_FACTOR = 0.11545F;
    constexpr const float STD_5V_3V3_CONVERSION_FACTOR = 0.641F;

    constexpr const float BIT_RESOLUTION = 4096.0F;
}

namespace ACUSystems
{
    constexpr const volt MIN_DISCHARGE_VOLTAGE_THRESH = 3.8F; // Minimum voltage for a cell to be discharged
    constexpr const volt CELL_OVERVOLTAGE_THRESH = 4.2;   // Cell overvoltage threshold in Volts
    constexpr const volt CELL_UNDERVOLTAGE_THRESH = 3.05; // Cell undervoltage threshold in Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0;  // Volts
    constexpr const celsius CHARGING_OT_THRESH = 60.0;    // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 60.0;     // Celsius
    constexpr const volt VOLTAGE_DIFF_TO_INIT_CB = 0.02;  // differential with lowest cell voltage to enable cell balancing for a cell
    constexpr const celsius BALANCE_TEMP_LIMIT_C = 50.0;
    constexpr const celsius BALANCE_ENABLE_TEMP_THRESH_C = 35.0; // Celsius
    constexpr const volt TS_ISOLATION_VOLTAGE = 100; // Volts
}
namespace ACUConstants
{
    constexpr size_t NUM_CELLS = 126;
    constexpr size_t NUM_CHIPS = 12;
    constexpr size_t NUM_CELL_TEMPS = 48;
    constexpr size_t NUM_CHIP_SELECTS = 2;

    const float VALID_SHDN_OUT_MIN_VOLTAGE_THRESHOLD = 12.0F;
    const uint32_t MIN_ALLOWED_INVALID_SHDN_OUT_MS = 10;  // 10 ms -- requies 100 Hz samp freq.

    // Initialize chip_select, chip_select_per_chip, and address
    constexpr std::array<int, NUM_CHIP_SELECTS> CS = {36, 38};
    constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {36, 36, 36, 36, 36, 36, 38, 38, 38, 38, 38, 38};
    constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

    /* Task Times */
    constexpr uint8_t IDLE_SAMPLE_PRIORITY = 0;
    constexpr time_us IDLE_SAMPLE_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz

    constexpr uint8_t WATCHDOG_PRIORITY = 1;
    constexpr time_us KICK_WATCHDOG_PERIOD_US = 4000UL; // 10 000 us = 100 Hz

    constexpr uint8_t SAMPLE_BMS_PRIORITY = 2;
    constexpr time_us SAMPLE_BMS_PERIOD_US = 2500UL; // 5 000 us = 200 Hz (since we are reading by group)

    constexpr uint8_t ACU_OK_CAN_PRIORITY = 3;
    constexpr time_us ACU_OK_CAN_PERIOD_US = 50000UL; // 50 000 us = 20 Hz

    constexpr uint8_t CORE_DATA_ETHERNET_PRIORITY = 4;
    constexpr time_us CORE_DATA_ETHERNET_PERIOD_US = 8000UL; // 8 000 us = 125 Hz

    constexpr uint8_t ALL_DATA_ETHERNET_PRIORITY = 5;
    constexpr time_us ALL_DATA_ETHERNET_PERIOD_US = 100000UL; // 100 000 us = 10 Hz

    constexpr uint8_t EM_MEASUREMENT_SEND_PRIORITY = 6;
    constexpr time_us EM_MEASUREMENT_SEND_PERIOD_US = 4000UL; // 4 000 us = 250 Hz

    constexpr uint8_t RECV_CAN_PRIORITY = 7;
    constexpr time_us RECV_CAN_PERIOD_US = 50000UL; // 50 000 us = 20 Hz

    constexpr uint8_t SEND_CAN_PRIORITY = 8;
    constexpr time_us SEND_CAN_PERIOD_US = 4000UL; // 40 000 us = 250 Hz

    constexpr uint8_t TICK_SM_PRIORITY = 9;
    constexpr time_us TICK_SM_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz

    constexpr uint8_t EVAL_ACC_PRIORITY = 10;
    constexpr time_us EVAL_ACC_PERIOD_US = 20000UL; // 20 000 us = 50 Hz (problem for soc if this is running faster than voltage)

    constexpr uint8_t SAMPLE_ADC_PRIORITY = 11;
    constexpr time_us SAMPLE_ADC_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz

    constexpr uint8_t CCU_SEND_A_PRIORITY = 12;
    constexpr time_us CCU_SEND_A_PERIOD_US = 100000UL; // 100 000 us = 10 Hz

    constexpr uint8_t CCU_SEND_B_PRIORITY = 13;
    constexpr time_us CCU_SEND_B_PERIOD_US = 100000UL; // 100 000 us = 10 Hz

    constexpr uint8_t CCU_SEND_PRIORITY = 14;
    constexpr time_us CCU_SEND_PERIOD_US = 100000UL; // 100 000 us = 10 Hz

    constexpr uint8_t WRITE_CELL_BALANCE_PRIORITY = 15;
    constexpr time_us WRITE_CELL_BALANCE_PERIOD_US = 100000UL; // 100 000 us = 10 Hz

    constexpr uint8_t DATA_LOG_PRIORITY = 18;
    constexpr time_us DATA_LOG_PERIOD_US = 500000UL; // 500 000 us = 2 Hz

    constexpr uint8_t SOH_PERSIST_PRIORITY = 19;
    constexpr time_us SOH_PERSIST_PERIOD_US = 1000000UL; // 1 000 000 us = 1 Hz

    constexpr uint8_t DEBUG_PRINT_PRIORITY = 20;
    constexpr time_us DEBUG_PRINT_PERIOD_US = 2000000UL; //250000UL; // 250 000 us = 4 Hz

    /* CAN Constants */
    const uint32_t VEH_CAN_BAUDRATE = 1000000;
    const uint32_t EM_CAN_BAUDRATE = 500000;
}

#endif
