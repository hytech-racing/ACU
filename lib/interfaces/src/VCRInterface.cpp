#include "VCRInterface.h"

#include "ACUCANInterfaceImpl.h"

void VCRInterface::set_monitoring_data(bool imd_ok, bool bms_ok) {
    _curr_data.imd_ok = imd_ok;
    _curr_data.bms_ok = bms_ok;
}

void VCRInterface::handle_enqueue_acu_ok_CAN_message()
{
    ACU_OK_t msg = {};
    msg.imd_ok = _curr_data.imd_ok;
    msg.bms_ok = _curr_data.bms_ok;
    CAN_util::enqueue_msg(&msg, &Pack_ACU_OK_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}