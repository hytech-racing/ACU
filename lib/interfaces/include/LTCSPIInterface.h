#ifndef LTCSPIINTERFFACE
#define LTCSPIINTERFFACE

/* Interface Includes */
#include <SPI.h>
#include "Configuration.h"
#include <stddef.h>
#include <array>

/**
 * Sends a SPI command to write data to the registers
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 24 bytes for _2 model
 * @param buffer 36 bytes of data, 6 bytes x 6 ICs of data
 * @param buffer_pec 12 bytes of data, 2 bytes x 6 ICs of PEC data
*/
template <size_t buffer_size>
void write_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec, const std::array<uint8_t, buffer_size> &data);

/**
 * Sends a SPI command to read registers
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 4 x 6 bytes for _2 model 
 * @return the data we read
*/
template <size_t buffer_size>
inline std::array<uint8_t, buffer_size> read_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec);

/**
 * Sends a SPI command to initiate some functionality of the device
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 24 bytes for _2 model 
*/
void adc_conversion_command(int cs, std::array<uint8_t, 4> cmd_and_pec, size_t num_stacked_devices);

#include <LTCSPIInterface.tpp>
#endif