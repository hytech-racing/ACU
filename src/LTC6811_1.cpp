/* Library Includes */
#include "LTC6811_1.h"
#include "DataContainer.h"

uint8_t* generate_broadcast_command(CMD_CODES_e cmd_code) {
    uint8_t cmd[2] = { ((uint8_t) cmd_code) >> 8, (uint8_t) cmd_code};
    return cmd;
}