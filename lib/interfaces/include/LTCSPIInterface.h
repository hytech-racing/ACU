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
void adc_conversion_command(int cs, std::array<uint8_t, 4> cmd_and_pec);

/**
 * Transfers bytes (uint8_t) of arbritrary length on to the SPI line
 * @param data input buffer of size length
 * @param length length of buffer, number of bytes
*/
template <size_t data_size>
void _transfer_SPI_data(const std::array<uint8_t, data_size> &data);

/**
 * Receives SPI data
 * @param length length of data we expect to receive
 * @return data we are receiving instantaneously
*/
template <size_t data_size>
std::array<uint8_t, data_size> _receive_SPI_data();

/**
 * Writes a LOW on one of the chip selects, then delays
 * @param cs the chip select we are writing to
 * @param delay time in microseconds
*/
void _write_and_delay_LOW(int cs, int delay_microSeconds);

/**
 * Writes a HIGH on one of the chip selects, then delay
 * @param cs the chip select we are writing to
 * @param delay time in microseconds
*/
void _write_and_delay_HIGH(int cs, int delay_microSeconds);

#include <LTCSPIInterface.tpp>
#endif