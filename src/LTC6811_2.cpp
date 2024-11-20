/* Library Includes */
#include "LTC6811_2.h"
#include "DataContainer.h"

uint8_t* generate_address_command(CMD_CODES_e cmd_code, int address) {
    uint8_t cmd[2] = {(uint8_t) (get_cmd_address(address) | (uint8_t) cmd_code >> 8), (uint8_t) cmd_code};
    return cmd;
}

uint8_t get_cmd_address(int address) { return 0x80 | (address << 3); } 
