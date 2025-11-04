template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void ACUCANInterface::acu_CAN_recv(
    CANInterfaces_s<num_cells, num_celltemps, num_chips> &interfaces,
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
