#ifndef __LTC6811_2_H__
#define __LTC6811_2_H__
/**
 * PREAMBLE: The LTC6811-2 can operate under both address and broadcast command formats. 
 * For the sake of file organization, THIS FILE will only be used to properly read and write address file
 * The point of addressed files is that they have an "address" embedded in the hardware (0 to 15)
 * 
 * We need to include this in our command format. The order of the dat bytes is not as relevant
 * Because only the IC with the correlating address will actually respond and understand the command
*/

/* Library Include */
#include "DataContainer.h"

/**
 * Generates a 2 byte command for addressable protocols
 * These bytes need to be structured as follows, reference page 58: 
 * Byte 1: [1, a3, a2, a1, a0, CC[10:8]]
 * Byte 2: CC[7:0] | where a# are the 4 bits for address and CC is the 12 bit command code
 * @param cmd_code the code we are writing
 * @param address input
 * @return uint8_t array of length 2 
*/
uint8_t* generate_address_command(CMD_CODES_e cmd_code, int address);

/**
 * Generates a 
*/

/**
 * @param address address of the IC we are writing command for
 * @return a usable command address of the specific LTC6811-2 chip to send command to
 */
uint8_t get_cmd_address(int address);

#endif