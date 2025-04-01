#include "CCUInterface.h"

#include "ACUCANInterface.h"

void CCUInterface::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    if (ccu_msg.charger_enabled == false || (_curr_data.charging_requested && ccu_msg.charger_enabled)) {
        _curr_data.last_time_charging_requested = curr_millis;
        Serial.println("Updated CCU Interface curr data.");
    } 
    _curr_data.charging_requested = (curr_millis - _curr_data.last_time_charging_requested) < _min_charging_enable_threshold;
}

void CCUInterface::handle_enqueue_acu_status_CAN_message() {
    BMS_STATUS_t msg = {};
    if (_curr_data.charging_requested == true) {
        msg.state = 2;
    } else {
        return;
    }
    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}  

void CCUInterface::handle_enqueue_acu_voltages_CAN_message() {
    BMS_VOLTAGES_t msg = {};
    msg.high_voltage_ro = _acu_core_data.max_cell_voltage;
    msg.low_voltage_ro = _acu_core_data.min_cell_voltage;
    msg.total_voltage_ro = _acu_core_data.pack_voltage;
    msg.average_voltage_ro = _acu_core_data.avg_cell_voltage;
    CAN_util::enqueue_msg(&msg, &Pack_BMS_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}       

CCUCANInterfaceData_s CCUInterface::get_latest_data() {
    return _curr_data;
}

void CCUInterface::set_ACU_core_data(ACUCoreData_s input) {
    _acu_core_data.avg_cell_voltage = input.avg_cell_voltage;
    _acu_core_data.max_cell_voltage = input.max_cell_voltage;
    _acu_core_data.min_cell_voltage = input.min_cell_voltage;
    _acu_core_data.pack_voltage = input.pack_voltage;
}
