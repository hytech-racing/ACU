#include "ACUCANInterface.h"
#include "ACUCANBuffers.h"
#include <array>
#include <cstring>

namespace ACUCANInterface {

// Define CAN hardware instances
FlexCAN_t<CAN3> CCU_CAN;
FlexCAN_t<CAN2> EM_CAN;

// CAN receive callback for CCU CAN bus
void on_ccu_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    std::memmove(buf.data(), &msg, sizeof(msg));
    ACUCANBuffers::ccu_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

// CAN receive callback for EM CAN bus
void on_em_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    std::memmove(buf.data(), &msg, sizeof(msg));
    ACUCANBuffers::em_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

// Send all queued CAN messages to the specified CAN interface
void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
{
    CAN_message_t msg;
    while (buffer.available())
    {
        std::array<uint8_t, sizeof(CAN_message_t)> buf;
        buffer.pop_front(buf.data(), sizeof(CAN_message_t));
        std::memmove(&msg, buf.data(), sizeof(msg));
        can_interface->write(msg);
    }
}

void acu_CAN_recv(
    CANInterfaces_s &interfaces,
    const CAN_message_t &msg,
    unsigned long millis)
{
    switch (msg.id)
    {
    case CCU_STATUS_CANID:
    {
        interfaces.ccu_interface.receive_CCU_status_message(msg, millis);
        break;
    }
    case EM_MEASUREMENT_CANID:
    {
        interfaces.em_interface.receive_EM_measurement_message(msg, millis);
        break;
    }
    default:
    {
        break;
    }
    }
}


} // namespace ACUCANInterface
