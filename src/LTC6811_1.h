#ifndef __LTC6811_1_H__
#define __LTC6811_1_H__
/**
 * PREAMBLE: The LTC6811-1 is only capable of communicating through broadcast commands
 * It reads and writes data with an appended set of data, called shift bytes according to the data sheet
 * These shift bytes are essentially the data from the following ICs adding their data to the front or end
 * Depending on if we're reading or writing, the newest data will be on the front or end, respectively
 * This requires a clear understanding of that data:
 * IDK yet
 * 
 * For us to record the data properly, we are using THIS FILE to pack and unpack the data.
*/

/* Library Includes */
#include "DataContainer.h"

/**
 * Generates a 2 byte command for broadcast protocols
 * These bytes need to be structured as follows, reference page 58: 
 * Byte 1: [0, 0, 0, 0, 0, CC[10:8]]
 * Byte 2: CC[7:0] | where CC is the 12 bit command code
 * @return uint8_t array of length 2 
*/
uint8_t* generate_broadcast_command(CMD_CODES_e command);

/**
 * 
*/

#endif