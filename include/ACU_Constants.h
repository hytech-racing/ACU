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
    constexpr size_t NUM_CELLS = 126;
    constexpr size_t NUM_CELL_TEMPS = 48;
    constexpr size_t NUM_CHIPS = 12;
    constexpr size_t NUM_BOARD_TEMPS = 12;
    constexpr size_t NUM_CHIP_SELECTS = 1;

    // Initialize chip_select, chip_select_per_chip, and address
    constexpr std::array<int, NUM_CHIP_SELECTS> CS = {10};
    constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
    constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

    /* Interface Constants */
    const size_t ANALOG_READ_RESOLUTION = 12;
    const size_t SERIAL_BAUDRATE = 115200;

    /* Task Times */
    constexpr uint32_t TICK_SM_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz
    constexpr uint32_t TICK_SM_PRIORITY = 9;
    constexpr uint32_t KICK_WATCHDOG_PERIOD_US = 100UL; // 100 us = 1 000 Hz | to compensate for delays, otherwise get 20ms watchdog periods->bad
    constexpr uint32_t WATCHDOG_PRIORITY = 1;
    constexpr uint32_t SAMPLE_BMS_PERIOD_US = 50000UL; // 50 000 us = 20Hz
    constexpr uint32_t SAMPLE_BMS_PRIORITY = 2;
    constexpr uint32_t EVAL_ACC_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t EVAL_ACC_PRIORITY = 10;
    constexpr uint32_t WRITE_CELL_BALANCE_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t WRITE_CELL_BALANCE_PRIORITY = 6;
    constexpr uint32_t ALL_DATA_ETHERNET_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t ALL_DATA_ETHERNET_PRIORITY = 5;
    constexpr uint32_t CORE_DATA_ETHERNET_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t CORE_DATA_ETHERNET_PRIORITY = 4;

    constexpr uint32_t CCU_SEND_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t CCU_SEND_PRIORITY = 11;
    constexpr uint32_t ACU_OK_CAN_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr uint32_t ACU_OK_CAN_PRIORITY = 3;
    constexpr uint32_t CCU_SEND_A_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t CCU_SEND_A_PRIORITY = 12;
    constexpr uint32_t CCU_SEND_B_PERIOD_US = 50000UL; // 50 000 us = 20 Hz
    constexpr uint32_t CCU_SEND_B_PRIORITY = 13; 

    constexpr uint32_t SEND_CAN_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr uint32_t SEND_CAN_PRIORITY = 8;
    constexpr uint32_t RECV_CAN_PERIOD_US = 1000UL; // 1 000 us = 1 000 Hz
    constexpr uint32_t RECV_CAN_PRIORITY = 0;

    constexpr uint32_t DEBUG_PRINT_PERIOD_US = 250000UL; // 250 000 us = 4 Hz
    constexpr uint32_t DEBUG_PRINT_PRIORITY = 7;

    /* Message Interface */
    const uint32_t Veh_CAN_baudrate = 1000000;
    const uint32_t EM_CAN_baudrate = 500000;
}

#endif