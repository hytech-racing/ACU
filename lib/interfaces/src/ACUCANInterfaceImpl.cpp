#include "ACUCANInterfaceImpl.h"

CANRXBuffer_t ACUCANInterfaceImpl::ccu_can_rx_buffer;
CANRXBuffer_t ACUCANInterfaceImpl::em_can_rx_buffer;
CANTXBuffer_t ACUCANInterfaceImpl::ccu_can_tx_buffer;

void ACUCANInterfaceImpl::on_ccu_can_receive(const CAN_message_t &msg)
{   
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    ccu_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void ACUCANInterfaceImpl::on_em_can_receive(const CAN_message_t &msg) 
{
    std::array<uint8_t, sizeof(CAN_message_t)> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    ccu_can_tx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
    em_can_rx_buffer.push_back(buf.data(), sizeof(CAN_message_t));
}

void ACUCANInterfaceImpl::acu_CAN_recv(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis)
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

void ACUCANInterfaceImpl::send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface)
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
