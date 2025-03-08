#include "ACU_InterfaceTasks.h"

void set_bit(uint16_t &value, uint8_t index, bool bitValue) {
    if (index >= 16) return; // Ensure index is within range

    if (bitValue) {
        value |= (1 << index);  // Set the bit
    } else {
        value &= ~(1 << index); // Clear the bit
    }
}