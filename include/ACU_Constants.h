#ifndef ACU_CONSTANTS
#define ACU_CONSTANTS

#include <cstddef>
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <array>
#include <algorithm>

namespace ACUConstants
{
    constexpr const volt OV_THRESH = 4.2; // Volts
    constexpr const volt UV_THRESH = 3.05; // Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0; // Volts
    constexpr const celsius CHARGING_OT_THRESH = 60.0; // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 60.0; // Celsius
    constexpr const size_t MAX_INVALID_PACKET_FAULT_COUNT = 1000000; // Same as voltage fault count
    constexpr const time_ms MAX_VOLTAGE_FAULT_DUR = 1000; // At 15 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const time_ms MAX_TEMP_FAULT_DUR = 1000; 
    constexpr const time_ms MAX_INVALID_PACKET_FAULT_DUR = 500; // In cases in EMI, we will need more leniency with invalid packet faults
    constexpr const volt VOLTAGE_DIFF_TO_INIT_CB = 0.02; // differential with lowest cell voltage to enable cell balancing for a cell
    constexpr const float PACK_NOMINAL_CAPACITY_AH = 13.5; // nominal pack capacity in amp * hours
    constexpr const float PACK_MAX_VOLTAGE = 529.2; // from data sheet https://wiki.hytechracing.org/books/ht09-design/page/molicel-pack-investigation
    constexpr const float PACK_MIN_VOLTAGE = 378.0; // from data sheet^ but just assume 126 * 3.0V
    constexpr const celsius BALANCE_TEMP_LIMIT_C = 50.0;
    constexpr const celsius BALANCE_ENABLE_TEMP_THRESH_C = 35.0; // Celsius
    constexpr const float PACK_INTERNAL_RESISTANCE = 0.246; // Ohms (measured)

    constexpr size_t NUM_CELLS = 126;
    constexpr size_t NUM_CELL_TEMPS = 48;
    constexpr size_t NUM_CHIPS = 12;
    constexpr size_t NUM_BOARD_TEMPS = 12;
    constexpr size_t NUM_CHIP_SELECTS = 2;

    // Initialize chip_select, chip_select_per_chip, and address
    constexpr std::array<int, NUM_CHIP_SELECTS> CS = {9, 10};
    constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10};
    constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

    /* Interface Constants */
    const size_t ANALOG_READ_RESOLUTION = 12;
    const size_t SERIAL_BAUDRATE = 115200;
    const float VALID_SHDN_OUT_MIN_VOLTAGE_THRESHOLD = 12.0F;
    const uint32_t MIN_ALLOWED_INVALID_SHDN_OUT_MS = 10;  // 10 ms -- requies 100 Hz samp freq.

    /* Task Times */
    constexpr uint32_t TICK_SM_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz
    constexpr uint32_t TICK_SM_PRIORITY = 9;
    constexpr uint32_t KICK_WATCHDOG_PERIOD_US = 100UL; // 100 us = 1 000 Hz | to compensate for delays, otherwise get 20ms watchdog periods->bad
    constexpr uint32_t WATCHDOG_PRIORITY = 1;
    constexpr uint32_t SAMPLE_BMS_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr uint32_t SAMPLE_BMS_PRIORITY = 2;
    constexpr uint32_t EVAL_ACC_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr uint32_t EVAL_ACC_PRIORITY = 10;
    constexpr uint32_t WRITE_CELL_BALANCE_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t WRITE_CELL_BALANCE_PRIORITY = 6;
    constexpr uint32_t ALL_DATA_ETHERNET_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t ALL_DATA_ETHERNET_PRIORITY = 5;
    constexpr uint32_t CORE_DATA_ETHERNET_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t CORE_DATA_ETHERNET_PRIORITY = 4;

    constexpr uint32_t CCU_SEND_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_PRIORITY = 11;
    constexpr uint32_t ACU_OK_CAN_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t ACU_OK_CAN_PRIORITY = 3;
    constexpr uint32_t CCU_SEND_A_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_A_PRIORITY = 12;
    constexpr uint32_t CCU_SEND_B_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_B_PRIORITY = 13;

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