/**
 * PREAMBLE:
 * This file will only contain 3 functions
 * 1. Sending SPI Commands for WRITING Registers
 * 2. Sending SPI Commands for READING Registers
 * 3. Sending SPI Commands to initiate device functionality, ADC conversions in our case 
*/
/* Interface Includes */
#include <SPI.h>
#include "Configuration.h"

/**
 * Sends a SPI command to write data to the registers
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 24 bytes for _2 model
 * @param buffer 36 bytes of data, 6 bytes x 6 ICs of data
 * @param buffer_pec 12 bytes of data, 2 bytes x 6 ICs of PEC data
*/
void send_SPI_write_registers_command(int cs, uint8_t* cmd_and_pec, uint8_t* buffer_and_pec);

/**
 * Sends a SPI command to read registers
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 4 x 6 bytes for _2 model 
 * @return the data we read
*/
uint8_t* send_SPI_read_registers_command(int cs, uint8_t* cmd_and_pec);

/**
 * Sends a SPI command to initiate some functionality of the device
 * @param cs chip select
 * @param cmd_and_pec 4 bytes if using _1 model, 24 bytes for _2 model 
*/
void send_SPI_non_register_command(int cs, uint8_t* cmd_and_pec);

/**
 * Transfers bytes (uint8_t) of arbritrary length on to the SPI line
 * @param data input buffer of size length
 * @param length length of buffer, number of bytes
*/
void transfer_SPI_data(uint8_t *data, int length);

/**
 * Receives SPI data
 * @param length length of data we expect to receive
 * @return data we are receiving instantaneously
*/
uint8_t* receive_SPI_data(int length);

/**
 * Writes a LOW on one of the chip selects, then delays
 * @param cs the chip select we are writing to
 * @param delay time in microseconds
*/
void write_and_delay_LOW(int cs, int delay_microSeconds);

/**
 * Writes a HIGH on one of the chip selects, then delay
 * @param cs the chip select we are writing to
 * @param delay time in microseconds
*/
void write_and_delay_HIGH(int cs, int delay_microSeconds);