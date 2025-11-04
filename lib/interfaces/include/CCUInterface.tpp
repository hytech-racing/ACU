#include "hytech.h"

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    
    if (ccu_msg.charger_enabled) _curr_data.last_time_charging_with_balancing_requested = curr_millis;
    _curr_data.prev_ccu_msg_recv_ms = curr_millis;
}

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::handle_enqueue_acu_status_CAN_message(bool charging_enabled) {
    BMS_STATUS_t msg = {};
    if (charging_enabled) {
        msg.charging_state = ACUChargingState_e::CHARGING;
    } else {
        msg.charging_state = ACUChargingState_e::NOT_CHARGING;
    }

    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}  

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::handle_enqueue_acu_voltage_statistics_CAN_message(
    float max_cell_voltage, float min_cell_voltage, float pack_voltage, float avg_cell_voltage
) {
    BMS_VOLTAGES_t msg = {};
    msg.max_cell_voltage_ro = HYTECH_max_cell_voltage_ro_toS(max_cell_voltage);
    msg.min_cell_voltage_ro = HYTECH_min_cell_voltage_ro_toS(min_cell_voltage);
    msg.total_voltage_ro = HYTECH_total_voltage_ro_toS(pack_voltage);
    msg.average_voltage_ro = HYTECH_average_voltage_ro_toS(avg_cell_voltage);
    CAN_util::enqueue_msg(&msg, &Pack_BMS_VOLTAGES_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::handle_enqueue_acu_cell_voltages_CAN_message(
    const std::array<volt, num_cells>& cell_voltages
) {
    BMS_CELL_VOLTAGES_t msg = {};
    msg.chip_id = static_cast<uint8_t>(_curr_data.current_voltage_group_chip_id);
    msg.cell_group_id = static_cast<uint8_t>(_curr_data.current_voltage_cell_group_id);

    msg.cell_group_voltage_0_ro = HYTECH_cell_group_voltage_0_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id]); 
    msg.cell_group_voltage_1_ro = HYTECH_cell_group_voltage_1_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id + 1]);
    msg.cell_group_voltage_2_ro = HYTECH_cell_group_voltage_2_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id + 2]);

    bool voltage_cell_group_cycle_loop_completed;
    if (_curr_data.current_voltage_group_chip_id % 2 == 0) {
        voltage_cell_group_cycle_loop_completed = _increment_and_loop_id(_curr_data.current_voltage_cell_group_id, _ccu_params.voltage_cell_groups_per_ic_even);
    } else {
        voltage_cell_group_cycle_loop_completed = _increment_and_loop_id(_curr_data.current_voltage_cell_group_id, _ccu_params.voltage_cell_groups_per_ic_odd);
    }

    if (voltage_cell_group_cycle_loop_completed) _increment_and_loop_id(_curr_data.current_voltage_group_chip_id, num_chips);

    _increment_and_loop_id(_curr_data.current_voltage_cell_id, num_cells, _ccu_params.voltage_cells_per_group);
    
    CAN_util::enqueue_msg(&msg, &Pack_BMS_CELL_VOLTAGES_hytech, ACUCANBuffers::ccu_can_tx_buffer);
} 
template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::handle_enqueue_acu_cell_temps_CAN_message(const std::array<celsius, num_celltemps>& cell_temps) {
    BMS_CHIP_TEMPS_t chip_temps_msg = {};

    chip_temps_msg.chip_id = static_cast<uint8_t>(_curr_data.current_temp_group_chip_id);
    chip_temps_msg.thermistor_group_id = static_cast<uint8_t>(_curr_data.current_temp_group_id);

    chip_temps_msg.thermistor_cell_group_temp_0_ro = HYTECH_thermistor_cell_group_temp_0_ro_toS(cell_temps[_curr_data.current_temp_cell_id]); 
    chip_temps_msg.thermistor_cell_group_temp_1_ro = HYTECH_thermistor_cell_group_temp_1_ro_toS(cell_temps[_curr_data.current_temp_cell_id + 1]);

    bool temp_cell_group_cycle_loop_completed;
    temp_cell_group_cycle_loop_completed = _increment_and_loop_id(_curr_data.current_temp_group_id, _ccu_params.temp_cell_groups_per_ic);

    if (temp_cell_group_cycle_loop_completed) _increment_and_loop_id(_curr_data.current_temp_group_chip_id, num_chips);

    _increment_and_loop_id(_curr_data.current_temp_cell_id, num_celltemps, _ccu_params.temp_cells_per_group);

    CAN_util::enqueue_msg(&chip_temps_msg, &Pack_BMS_CHIP_TEMPS_hytech, ACUCANBuffers::ccu_can_tx_buffer);


} 

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
void CCUInterface<num_cells, num_celltemps, num_chips>::handle_enqueue_acu_temp_statistics_CAN_message(
    celsius max_board_temp,
    celsius max_cell_temp,
    celsius min_cell_temp,
    const std::array<celsius, num_chips>& board_temps
) {
    BMS_ONBOARD_TEMPS_t board_temp_msg = {};
    board_temp_msg.max_board_temp_ro = HYTECH_max_board_temp_ro_toS(max_board_temp);
    board_temp_msg.max_cell_temp_ro = HYTECH_max_cell_temp_ro_toS(max_cell_temp);
    board_temp_msg.min_cell_temp_ro = HYTECH_min_cell_temp_ro_toS(min_cell_temp);
    CAN_util::enqueue_msg(&board_temp_msg, &Pack_BMS_ONBOARD_TEMPS_hytech, ACUCANBuffers::ccu_can_tx_buffer);

    BMS_ONBOARD_CURRENT_TEMP_t current_board_temp_msg = {};
    current_board_temp_msg.chip_id = _curr_data.current_temp_board_id;
    current_board_temp_msg.temp_0_ro = HYTECH_temp_0_ro_toS(board_temps[_curr_data.current_temp_board_id]);
    _increment_and_loop_id(_curr_data.current_temp_board_id, num_chips);
    CAN_util::enqueue_msg(&current_board_temp_msg, &Pack_BMS_ONBOARD_CURRENT_TEMP_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
CCUCANInterfaceData_s CCUInterface<num_cells, num_celltemps, num_chips>::get_latest_data() {
    return _curr_data;
}

