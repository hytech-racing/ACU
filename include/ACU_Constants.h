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
    constexpr unsigned long KICK_WATCHDOG_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz | to compensate for delays, otherwise get 20ms watchdog periods
    constexpr unsigned long WATCHDOG_PRIORITY = 1;
    constexpr unsigned long SAMPLE_BMS_PERIOD_US = 200000UL; // 200 000 us = 5Hz
    constexpr unsigned long SAMPLE_BMS_PRIORITY = 2;
    constexpr unsigned long EVAL_ACC_PERIOD_US = 10000UL; // 10 000 us = 100 Hz
    constexpr unsigned long EVAL_ACC_PRIORITY = 3;
    constexpr unsigned long WRITE_CELL_BALANCE_PERIOD_US = 10000UL; // 200 000 us = 5 Hz
    constexpr unsigned long WRITE_CELL_BALANCE_PRIORITY = 4;
    constexpr unsigned long DEBUG_PRINT_PERIOD_US = 250000UL; // 250 000 us = 4 Hz
    constexpr unsigned long DEBUG_PRINT_PRIORITY = 8;

    /* Message Interface */
    const uint32_t CAN_baudrate = 500000;
}

#endif