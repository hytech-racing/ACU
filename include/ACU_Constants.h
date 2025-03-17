#ifndef ACU_CONSTANTS
#define ACU_CONSTANTS

#include <cstddef>
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <array>

constexpr size_t NUM_CELLS = 126;
constexpr size_t NUM_CELL_TEMPS = 48;
constexpr size_t NUM_CHIPS = 12;
constexpr size_t NUM_CHIP_SELECTS = 1;

// Initialize chip_select, chip_select_per_chip, and address
constexpr std::array<int, NUM_CHIP_SELECTS> CS = {10};
constexpr std::array<int, NUM_CHIPS> CS_PER_CHIP = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
constexpr std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

/* Task Times */
constexpr unsigned long WATCHDOG_KICK_INTERVAL = 10;                 // 10 ms = 100 Hz

/* Message Interface */
const uint32_t CAN_baudrate = 500000;

#endif