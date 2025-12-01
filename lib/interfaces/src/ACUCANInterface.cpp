#include "ACUCANInterface.h"
#include "ACUCANBuffers.h"
#include <array>
#include <cstring>


void ACUCANInterface::on_ccu_can_receive(const CAN_message_t &msg)
{   
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    ACUCANBuffers::ccu_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void ACUCANInterface::on_em_can_receive(const CAN_message_t &msg) 
{
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    ACUCANBuffers::ccu_can_tx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
    ACUCANBuffers::em_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void ACUCANInterface::acu_CAN_recv(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis)
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

void ACUCANInterface::send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
{
    CAN_message_t msg;
    while (buffer.available())
    {
        std::array<uint8_t, sizeof(CAN_message_t)> buf;
        buffer.pop_front(buf.data(), sizeof(CAN_message_t));
        memmove(&msg, buf.data(), sizeof(msg));
        can_interface->write(msg);
    }
}