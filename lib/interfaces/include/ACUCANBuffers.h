#ifndef ACU_CAN_BUFFERS_H
#define ACU_CAN_BUFFERS_H

#include <cstdint>
#include "FlexCAN_T4.h"
#include "CANInterface.h"

const size_t CAN_MSG_SIZE = sizeof(CAN_message_t);
using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CAN_MSG_SIZE>;
using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CAN_MSG_SIZE>;

namespace ACUCANBuffers {
    extern CANRXBuffer_t ccu_can_rx_buffer;
    extern CANRXBuffer_t em_can_rx_buffer;
    extern CANTXBuffer_t ccu_can_tx_buffer;
}

#endif // CAN_BUFFERS_H
