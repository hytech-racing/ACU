#include "hytech.h"
#include "CCUInterface.h"

void CCUInterface::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    
    if (ccu_msg.charger_enabled) _curr_data.last_time_charging_with_balancing_requested = curr_millis;
    _curr_data.prev_ccu_msg_recv_ms = curr_millis;
}

void CCUInterface::handle_enqueue_acu_status_CAN_message(bool shdn_out_voltage_high) {
    BMS_STATUS_t msg = {};
        msg.shdn_out_voltage_state = shdn_out_voltage_high;
    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}  

void CCUInterface::handle_enqueue_acu_voltage_statistics_CAN_message(
    float max_cell_voltage, float min_cell_voltage, float pack_voltage, float avg_cell_voltage
) {
    BMS_VOLTAGES_t msg = {};
    msg.max_cell_voltage_ro = HYTECH_max_cell_voltage_ro_toS(max_cell_voltage);
    msg.min_cell_voltage_ro = HYTECH_min_cell_voltage_ro_toS(min_cell_voltage);
    msg.total_voltage_ro = HYTECH_total_voltage_ro_toS(pack_voltage);
    msg.average_voltage_ro = HYTECH_average_voltage_ro_toS(avg_cell_voltage);
    CAN_util::enqueue_msg(&msg, &Pack_BMS_VOLTAGES_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}

void CCUInterface::handle_enqueue_acu_cell_voltages_CAN_message(
    const volt* cell_voltages, 
    const size_t* voltage_cells_per_chip,
    const size_t num_of_chips
) {
    BMS_CELL_VOLTAGES_t msg = {};
    msg.chip_id = static_cast<uint8_t>(_curr_data.current_voltage_group_chip_id);
    msg.cell_group_id = static_cast<uint8_t>(_curr_data.current_voltage_cell_group_id);

    size_t num_of_groups = voltage_cells_per_chip[_curr_data.current_voltage_group_chip_id] / _ccu_params.voltage_cells_per_group; //NOLINT
    size_t num_of_voltage_cells = std::accumulate(voltage_cells_per_chip, voltage_cells_per_chip + num_of_chips, 0); //NOLINT

    msg.cell_group_voltage_0_ro = HYTECH_cell_group_voltage_0_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id]); //NOLINT
    msg.cell_group_voltage_1_ro = HYTECH_cell_group_voltage_1_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id + 1]); //NOLINT
    msg.cell_group_voltage_2_ro = HYTECH_cell_group_voltage_2_ro_toS(cell_voltages[_curr_data.current_voltage_cell_id + 2]); //NOLINT

    size_t voltage_cell_group_cycle_loop_completed = _increment_and_loop_id(_curr_data.current_voltage_cell_group_id, num_of_groups);

    if (voltage_cell_group_cycle_loop_completed) _increment_and_loop_id(_curr_data.current_voltage_group_chip_id, num_of_chips);

    _increment_and_loop_id(_curr_data.current_voltage_cell_id, num_of_voltage_cells, _ccu_params.voltage_cells_per_group);
    
    CAN_util::enqueue_msg(&msg, &Pack_BMS_CELL_VOLTAGES_hytech, ACUCANBuffers::ccu_can_tx_buffer);
} 
void CCUInterface::handle_enqueue_acu_cell_temps_CAN_message(
    const celsius* cell_temps, 
    const size_t* temp_cells_per_chip,
    const size_t num_of_chips
) {
    BMS_CHIP_TEMPS_t chip_temps_msg = {};

    chip_temps_msg.chip_id = static_cast<uint8_t>(_curr_data.current_temp_group_chip_id);
    chip_temps_msg.thermistor_group_id = static_cast<uint8_t>(_curr_data.current_temp_group_id);

    size_t num_of_groups = temp_cells_per_chip[_curr_data.current_temp_group_chip_id] / _ccu_params.temp_cells_per_group; //NOLINT
    size_t num_of_temp_cells = std::accumulate(temp_cells_per_chip, temp_cells_per_chip + num_of_chips, 0); //NOLINT

    chip_temps_msg.thermistor_cell_group_temp_0_ro = HYTECH_thermistor_cell_group_temp_0_ro_toS(cell_temps[_curr_data.current_temp_cell_id]); //NOLINT
    chip_temps_msg.thermistor_cell_group_temp_1_ro = HYTECH_thermistor_cell_group_temp_1_ro_toS(cell_temps[_curr_data.current_temp_cell_id + 1]); //NOLINT

    bool temp_cell_group_cycle_loop_completed = _increment_and_loop_id(_curr_data.current_temp_group_id, num_of_groups);

    if (temp_cell_group_cycle_loop_completed) _increment_and_loop_id(_curr_data.current_temp_group_chip_id, num_of_chips);

    _increment_and_loop_id(_curr_data.current_temp_cell_id, num_of_temp_cells, _ccu_params.temp_cells_per_group);

    CAN_util::enqueue_msg(&chip_temps_msg, &Pack_BMS_CHIP_TEMPS_hytech, ACUCANBuffers::ccu_can_tx_buffer);


} 

void CCUInterface::handle_enqueue_acu_temp_statistics_CAN_message(
    celsius max_board_temp,
    celsius max_cell_temp,
    celsius min_cell_temp
) {
    BMS_ONBOARD_TEMPS_t board_temp_msg = {};
    board_temp_msg.max_board_temp_ro = HYTECH_max_board_temp_ro_toS(max_board_temp);
    board_temp_msg.max_cell_temp_ro = HYTECH_max_cell_temp_ro_toS(max_cell_temp);
    board_temp_msg.min_cell_temp_ro = HYTECH_min_cell_temp_ro_toS(min_cell_temp);
    CAN_util::enqueue_msg(&board_temp_msg, &Pack_BMS_ONBOARD_TEMPS_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}

void CCUInterface::handle_enqueue_acu_board_temps_CAN_message(
    const celsius* board_temps,
    const size_t num_of_boards
) {
    BMS_ONBOARD_CURRENT_TEMP_t current_board_temp_msg = {};
    current_board_temp_msg.chip_id = _curr_data.current_temp_board_id;
    current_board_temp_msg.temp_0_ro = HYTECH_temp_0_ro_toS(board_temps[_curr_data.current_temp_board_id]); //NOLINT
    _increment_and_loop_id(_curr_data.current_temp_board_id, num_of_boards);
    CAN_util::enqueue_msg(&current_board_temp_msg, &Pack_BMS_ONBOARD_CURRENT_TEMP_hytech, ACUCANBuffers::ccu_can_tx_buffer);
}

CCUCANInterfaceData_s CCUInterface::get_latest_data() {
    return _curr_data;
}

