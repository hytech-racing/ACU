#include "CCUInterface.h"

#include "ACUCANInterface.h"

void CCUInterface::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    if (ccu_msg.charger_enabled == true) {
        _curr_data.charging_requested = (curr_millis - _curr_data.last_time_charging_requested) < _min_charging_enable_threshold;
        _curr_data.last_time_charging_requested = curr_millis;
        Serial.println("Updated CCU Interface curr data.");
    } else {
        _curr_data.charging_requested = false;
    }
}

void CCUInterface::handle_enqueue_acu_CAN_message() {
    BMS_STATUS_t msg = {};
    if (_curr_data.charging_requested == true) {
        msg.state = 2;
    } else {
        return;
    }
    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}       

CCUCANInterfaceData_s CCUInterface::get_latest_data() {
    return _curr_data;
}