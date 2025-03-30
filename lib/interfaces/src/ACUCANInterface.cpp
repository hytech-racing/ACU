#include "ACUCANInterface.h"

CANRXBufferType ACUCANInterfaceImpl::ccu_can_rx_buffer;
CANTXBufferType ACUCANInterfaceImpl::ccu_can_tx_buffer;

void ACUCANInterfaceImpl::on_ccu_can_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    ccu_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
}

template <size_t num_cells, size_t num_celltemps>
void ACUCANInterfaceImpl::acu_CAN_recv(ACUData_s<num_cells, num_celltemps> &acu_data, const CAN_message_t &msg, unsigned long millis)
{
    switch (msg.id)
    {
    case CCU_STATUS_CANID:
    {
        // check if status says charging_enabled = false;
        acu_data.charging_enabled = true;
        Serial.println("Message with CCU ID Received");
        break;
    }
    default:
    {
        acu_data.charging_enabled = false;
        break;
    }
    }
}

void ACUCANInterfaceImpl::send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface)
{
    CAN_message_t msg;
    while (buffer.available())
    {
        CAN_message_t msg;
        uint8_t buf[sizeof(CAN_message_t)];
        buffer.pop_front(buf, sizeof(CAN_message_t));
        memmove(&msg, buf, sizeof(msg));
        can_interface->write(msg);
    }
}
