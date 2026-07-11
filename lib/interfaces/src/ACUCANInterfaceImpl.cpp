#include "ACUCANInterfaceImpl.h"


void ACUCANInterfaceImpl::on_ccu_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    ACUCANInterfaceInstance::instance().ccu_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
}

void ACUCANInterfaceImpl::on_em_can_receive(const CAN_message_t &msg)
{
    std::array<uint8_t, CAN_MSG_SIZE> buf;
    memmove(buf.data(), &msg, CAN_MSG_SIZE);
    ACUCANInterfaceInstance::instance().ccu_can_tx_buffer.push_back(buf.data(), CAN_MSG_SIZE);
    ACUCANInterfaceInstance::instance().em_can_rx_buffer.push_back(buf.data(), CAN_MSG_SIZE);

}

void ACUCANInterfaceImpl::acu_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type)
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
        std::array<uint8_t, CAN_MSG_SIZE> buf;
        buffer.pop_front(buf.data(), CAN_MSG_SIZE);
        memmove(&msg, buf.data(), CAN_MSG_SIZE);
        can_interface->write(msg);
    }
}
