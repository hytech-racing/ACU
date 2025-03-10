#ifndef ACU_CONSTANTS
#define ACU_CONSTANTS

#include <cstddef>
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <array>

constexpr size_t NUM_CELLS = 126;
constexpr size_t NUM_CHIPS = 12;
constexpr size_t NUM_CHIP_SELECTS = 1;

// Initialize chip_select, chip_select_per_chip, and address
std::array<int, NUM_CHIP_SELECTS> CS = {10};
std::array<int, NUM_CHIPS> CS_PER_CHIP = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
std::array<int, NUM_CHIPS> ADDR = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // only for addressable bms chips

#endif