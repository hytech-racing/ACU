#ifndef ACU_CONSTANTS
#define ACU_CONSTANTS

#include <cstddef>
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <array>
#include <algorithm>

using volt = float;
using celsius = float;
using time_ms = uint32_t;

namespace ACUSystems
{
    constexpr const volt CELL_OVERVOLTAGE_THRESH = 4.2;   // Cell overvoltage threshold in Volts
    constexpr const volt CELL_UNDERVOLTAGE_THRESH = 3.05; // Cell undervoltage threshold in Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0;  // Volts
    constexpr const celsius CHARGING_OT_THRESH = 60.0;    // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 60.0;     // Celsius
    constexpr const volt VOLTAGE_DIFF_TO_INIT_CB = 0.02;  // differential with lowest cell voltage to enable cell balancing for a cell
    constexpr const celsius BALANCE_TEMP_LIMIT_C = 50.0;
    constexpr const celsius BALANCE_ENABLE_TEMP_THRESH_C = 35.0; // Celsius
}

namespace ACUInterfaces {
    /* Interface Constants */
    const size_t ANALOG_READ_RESOLUTION = 12;
    const size_t SERIAL_BAUDRATE = 115200;

    constexpr const size_t TEENSY_OK_PIN = 3; // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    constexpr const size_t WD_KICK_PIN = 4;       // > Needs to flip at 100 Hz to keep BMS_OK high
    constexpr const size_t N_LATCH_EN_PIN = 6;    // > Input to Safety Light, true when teensy is not in FAULT state
    constexpr const size_t TS_OUT_FILTERED_PIN = 17;
    constexpr const size_t PACK_OUT_FILTERED_PIN = 18;
    constexpr const size_t IMD_OK_PIN = 25; // < READ from IMD hardware, go to FAULT state if HIGH
    constexpr const size_t PRECHARGE_PIN = 26; // READ from PRECHARGE
    constexpr const size_t BSPD_CURRENT_PIN = 27;
    constexpr const size_t SHDN_OUT_PIN = 38; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    constexpr const size_t SCALED_24V_PIN = 39;

    constexpr const float SHUTDOWN_CONV_FACTOR = 0.1155F; // voltage divider -> 4.7k / (4.7k + 36k)
    constexpr const float PRECHARGE_CONV_FACTOR = 0.6623F; // voltage divider -> 10k / (5.1k + 10k)
    constexpr const float PACK_AND_TS_OUT_CONV_FACTOR = 0.00482F;
    constexpr const float SHDN_OUT_CONV_FACTOR = 0.1036F;
    constexpr const float BSPD_CURRENT_CONV_FACTOR = 0.5118F;
    constexpr const float GLV_CONV_FACTOR = 0.1036F;

    constexpr const float BIT_RESOLUTION = 4095.0F;
}

namespace ACUConstants
{
    constexpr size_t NUM_CELLS = 126;
    constexpr size_t NUM_CELL_TEMPS = 48;
    constexpr size_t NUM_CHIPS = 12;
    constexpr size_t NUM_BOARD_TEMPS = 12;
    constexpr size_t NUM_CHIP_SELECTS = 2;
    constexpr std::array<size_t, NUM_CHIPS> CELLS_PER_CHIP = {12, 9, 12, 9, 12, 9, 12, 9, 12, 9, 12, 9};
    constexpr std::array<size_t, NUM_CHIPS> TEMP_CELLS_PER_CHIP = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
    
    const float VALID_SHDN_OUT_MIN_VOLTAGE_THRESHOLD = 12.0F;
    const uint32_t MIN_ALLOWED_INVALID_SHDN_OUT_MS = 10;  // 10 ms -- requies 100 Hz samp freq.

    // Initialize chip_select, chip_select_per_chip, and address
    constexpr std::array<int, NUM_CHIP_SELECTS> CS = {9, 10};
    constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10};
    constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

    /* Task Times */
    constexpr uint32_t TICK_SM_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz
    constexpr uint32_t TICK_SM_PRIORITY = 9;
    constexpr uint32_t KICK_WATCHDOG_PERIOD_US = 5000UL; // 5000 us = 200 Hz
    constexpr uint32_t WATCHDOG_PRIORITY = 1;
    constexpr uint32_t SAMPLE_BMS_PERIOD_US = 4000UL; // 4 000 us = 3250 Hz (since we are reading by group)
    constexpr uint32_t SAMPLE_BMS_PRIORITY = 2;
    constexpr uint32_t EVAL_ACC_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr uint32_t EVAL_ACC_PRIORITY = 10;
    constexpr uint32_t WRITE_CELL_BALANCE_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t WRITE_CELL_BALANCE_PRIORITY = 6;
    constexpr uint32_t ALL_DATA_ETHERNET_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t ALL_DATA_ETHERNET_PRIORITY = 5;
    constexpr uint32_t CORE_DATA_ETHERNET_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr uint32_t CORE_DATA_ETHERNET_PRIORITY = 4;

    constexpr uint32_t CCU_SEND_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_PRIORITY = 11;
    constexpr uint32_t ACU_OK_CAN_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t ACU_OK_CAN_PRIORITY = 3;
    constexpr uint32_t CCU_SEND_A_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_A_PRIORITY = 12;
    constexpr uint32_t CCU_SEND_B_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_B_PRIORITY = 13;
    constexpr uint32_t CCU_SEND_C_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_C_PRIORITY = 14;

    constexpr uint32_t SEND_CAN_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t SEND_CAN_PRIORITY = 8;
    constexpr uint32_t RECV_CAN_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t RECV_CAN_PRIORITY = 7;

    constexpr uint32_t DEBUG_PRINT_PERIOD_US = 250000UL; // 250 000 us = 4 Hz
    constexpr uint32_t DEBUG_PRINT_PRIORITY = 20;

    constexpr uint32_t IDLE_SAMPLE_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz
    constexpr uint32_t IDLE_SAMPLE_PRIORITY = 0;

    /* Message Interface */
    const uint32_t Veh_CAN_baudrate = 1000000;
    const uint32_t EM_CAN_baudrate = 500000;
}

#endif