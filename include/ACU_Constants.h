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
    constexpr size_t NUM_CHIP_SELECTS = 1;

    // Initialize chip_select, chip_select_per_chip, and address
    constexpr std::array<int, NUM_CHIP_SELECTS> CS = {10};
    constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
    constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

    /* Task Times */
    constexpr unsigned long TICK_SM_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz
    constexpr unsigned long TICK_SM_PRIORITY = 0;
    constexpr unsigned long KICK_WATCHDOG_PERIOD_US = 2000UL; // 2 000 us = 500 Hz | to compensate for delays, otherwise get 20ms watchdog periods->bad
    constexpr unsigned long WATCHDOG_PRIORITY = 1;
    constexpr unsigned long SAMPLE_BMS_PERIOD_US = 200000UL; // 200 000 us = 5Hz
    constexpr unsigned long SAMPLE_BMS_PRIORITY = 2;
    constexpr unsigned long EVAL_ACC_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr unsigned long EVAL_ACC_PRIORITY = 3;
    constexpr unsigned long WRITE_CELL_BALANCE_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr unsigned long WRITE_CELL_BALANCE_PRIORITY = 4;
    constexpr unsigned long ALL_DATA_ETHERNET_PERIOD_US = 100000UL; // 100 000 us = 10 Hz
    constexpr unsigned long ALL_DATA_ETHERNET_PRIORITY = 5;
    constexpr unsigned long CORE_DATA_ETHERNET_PERIOD_US = 200000UL; // 200 000 us = 5 Hz
    constexpr unsigned long CORE_DATA_ETHERNET_PRIORITY = 6;
    constexpr unsigned long CCU_SEND_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr unsigned long CCU_SEND_PRIORITY = 7;
    constexpr unsigned long SEND_CAN_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr unsigned long SEND_CAN_PRIORITY = 8;
    constexpr unsigned long RECV_CAN_PERIOD_US = 20000UL; // 20 000 us = 50 Hz
    constexpr unsigned long RECV_CAN_PRIORITY = 9;

    constexpr unsigned long DEBUG_PRINT_PERIOD_US = 250000UL; // 250 000 us = 4 Hz
    constexpr unsigned long DEBUG_PRINT_PRIORITY = 10;

    /* Message Interface */
    const uint32_t CAN_baudrate = 500000;
}

#endif