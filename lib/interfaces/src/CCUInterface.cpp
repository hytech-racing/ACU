#include "CCUInterface.h"

#include "ACUCANInterfaceImpl.h"
#include "hytech.h"

void CCUInterface::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    if (ccu_msg.charger_enabled == false || (_curr_data.charging_requested && ccu_msg.charger_enabled)) {
        _curr_data.last_time_charging_requested = curr_millis;
    } 
    _curr_data.is_connected_to_CCU = (curr_millis - _curr_data.prev_ccu_msg_recv_ms) < _ccu_params._min_charging_enable_threshold;
    _curr_data.prev_ccu_msg_recv_ms = curr_millis;
}

void CCUInterface::handle_enqueue_acu_status_CAN_message() {
    BMS_STATUS_t msg = {};
    if (_curr_data.charging_requested) {
        msg.state = 2; // charging
    } else {
        msg.state = 1; // discharging
    }

    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}  

void CCUInterface::handle_enqueue_acu_core_voltages_CAN_message() {
    BMS_VOLTAGES_t msg = {};
    msg.high_voltage_ro = HYTECH_high_voltage_ro_toS(_acu_core_data.max_cell_voltage);
    msg.low_voltage_ro = HYTECH_low_voltage_ro_toS(_acu_core_data.min_cell_voltage); 
    msg.total_voltage_ro = HYTECH_total_voltage_ro_toS(_acu_core_data.pack_voltage);
    msg.average_voltage_ro = HYTECH_average_voltage_ro_toS(_acu_core_data.avg_cell_voltage);
    CAN_util::enqueue_msg(&msg, &Pack_BMS_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}

void CCUInterface::handle_enqueue_acu_voltages_CAN_message() {
    BMS_DETAILED_VOLTAGES_t detailed_msg = {};
    detailed_msg.ic_id = static_cast<uint8_t>(_curr_data.detailed_voltages_ic_id);
    detailed_msg.group_id = static_cast<uint8_t>(_curr_data.detailed_voltages_group_id);
    detailed_msg.voltage_0_ro = HYTECH_voltage_0_ro_toS(_acu_all_data.cell_voltages[_curr_data.detailed_voltages_cell_id]); 
    detailed_msg.voltage_1_ro = HYTECH_voltage_1_ro_toS(_acu_all_data.cell_voltages[_curr_data.detailed_voltages_cell_id+1]); 
    detailed_msg.voltage_2_ro = HYTECH_voltage_2_ro_toS(_acu_all_data.cell_voltages[_curr_data.detailed_voltages_cell_id+2]); 

    if (_curr_data.detailed_voltages_ic_id % 2 == 0) {
        _curr_data.detailed_voltages_group_id = (_curr_data.detailed_voltages_group_id == 3) ? 0 : _curr_data.detailed_voltages_group_id+1;
    } else {
        _curr_data.detailed_voltages_group_id = (_curr_data.detailed_voltages_group_id == 2) ? 0 : _curr_data.detailed_voltages_group_id+1;
    }
    if (_curr_data.detailed_voltages_group_id == 0) {
        _curr_data.detailed_voltages_ic_id = (_curr_data.detailed_voltages_ic_id == (ccu_interface_defaults::NUM_CHIPS - 1)) ? 0 : _curr_data.detailed_voltages_ic_id+1;
    }
    _curr_data.detailed_voltages_cell_id = (_curr_data.detailed_voltages_cell_id == ccu_interface_defaults::NUM_CELLS - 3) ? 0 : _curr_data.detailed_voltages_cell_id+3;
    
    CAN_util::enqueue_msg(&detailed_msg, &Pack_BMS_DETAILED_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
} 

void CCUInterface::handle_enqueue_acu_temps_CAN_message() {
    BMS_DETAILED_TEMPS_t detailed_msg = {};
    detailed_msg.ic_id = static_cast<uint8_t>(_curr_data.detailed_temps_ic_id);
    detailed_msg.group_id = static_cast<uint8_t>(_curr_data.detailed_temps_group_id);
    detailed_msg.thermistor_id_0_ro = HYTECH_thermistor_id_0_ro_toS(_acu_all_data.cell_temps[_curr_data.detailed_temps_cell_id]); 
    detailed_msg.thermistor_id_1_ro = HYTECH_thermistor_id_1_ro_toS(_acu_all_data.cell_temps[_curr_data.detailed_temps_cell_id+1]); 
    detailed_msg.thermistor_id_2_ro = HYTECH_thermistor_id_2_ro_toS(_acu_all_data.cell_temps[_curr_data.detailed_temps_cell_id+2]); 
    
    _curr_data.detailed_temps_group_id = (_curr_data.detailed_temps_group_id == 1) ? 0 : _curr_data.detailed_temps_group_id+1;
    if (_curr_data.detailed_temps_group_id == 0) {
        _curr_data.detailed_temps_ic_id = (_curr_data.detailed_temps_ic_id == (ccu_interface_defaults::NUM_CHIPS - 1)) ? 0 : _curr_data.detailed_temps_ic_id+1;
    }
    _curr_data.detailed_temps_cell_id = (_curr_data.detailed_temps_cell_id == (ccu_interface_defaults::NUM_CELLTEMPS - 3)) ? 0 : _curr_data.detailed_temps_cell_id+3;
    CAN_util::enqueue_msg(&detailed_msg, &Pack_BMS_DETAILED_TEMPS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);

    BMS_ONBOARD_TEMPS_t board_temp_msg = {};
    board_temp_msg.max_board_temp_ro = HYTECH_max_board_temp_ro_toS(_acu_all_data.core_data.max_board_temp);
    board_temp_msg.high_cell_temp_ro = HYTECH_high_cell_temp_ro_toS(_acu_all_data.core_data.max_cell_temp);
    board_temp_msg.low_cell_temp_ro = HYTECH_low_cell_temp_ro_toS(_acu_all_data.core_data.min_cell_temp);
    CAN_util::enqueue_msg(&board_temp_msg, &Pack_BMS_ONBOARD_TEMPS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);

    BMS_ONBOARD_DETAILED_TEMPS_t detailed_board_temp_msg = {};
    detailed_board_temp_msg.ic_id = _curr_data.detailed_temps_board_id;
    detailed_board_temp_msg.temp_0_ro = HYTECH_temp_0_ro_toS(_acu_all_data.board_temps[_curr_data.detailed_temps_board_id]);
    _curr_data.detailed_temps_board_id = (_curr_data.detailed_temps_board_id == (ccu_interface_defaults::NUM_CHIPS - 1)) ? 0 : _curr_data.detailed_temps_board_id+1;
    CAN_util::enqueue_msg(&detailed_board_temp_msg, &Pack_BMS_ONBOARD_DETAILED_TEMPS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
} 

void CCUInterface::set_system_latch_state(unsigned long curr_millis, bool is_latched) {
    _curr_data.charging_requested = is_latched && ((curr_millis - _curr_data.last_time_charging_requested) < _ccu_params._min_charging_enable_threshold);
}  

CCUCANInterfaceData_s CCUInterface::get_latest_data(unsigned long curr_millis) {
    return _curr_data;
}

