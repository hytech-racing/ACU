#include "ACUController.h"

template <size_t num_cells>
void ACUController<num_cells>::init(time_ms system_start_time) {
    _acu_state.last_time_ov_fault_not_present = system_start_time;
    _acu_state.last_time_uv_fault_not_present = system_start_time;
    _acu_state.last_time_board_ot_fault_not_present = system_start_time;
    _acu_state.last_time_cell_ot_fault_not_present = system_start_time;
    _acu_state.last_time_pack_uv_fault_not_present = system_start_time;
    _acu_state.last_time_invalid_packet_present = system_start_time;
}

template <size_t num_cells>
typename ACUController<num_cells>::ACUStatus
ACUController<num_cells>::evaluate_accumulator(time_ms current_millis, const ACUData_s<num_cells> &input_state)
{   
    _acu_state.charging_enabled = false; // comment after CCU CAN read implemented

    bool has_invalid_packet = false;
    if (input_state.global_invalid_packet_count != 0) { // meaning that all of the data is valid
        has_invalid_packet = true;
    }

    // Cell balancing calculations
    if (_acu_state.charging_enabled)
    {
        _acu_state.cb = _calculate_cell_balance_statuses(input_state.voltages, input_state.min_cell_voltage);
    } else {
        // Fill with zeros, no balancing
        _acu_state.cb.fill(0);
    }

    // Update voltage fault time stamps
    if (input_state.max_cell_voltage < _parameters.ov_thresh_v || has_invalid_packet) { 
        _acu_state.last_time_ov_fault_not_present = current_millis;
    } 
    if (input_state.min_cell_voltage > _parameters.uv_thresh_v || has_invalid_packet) { 
        _acu_state.last_time_uv_fault_not_present = current_millis;
    }
    if (input_state.pack_voltage > _parameters.min_pack_total_v || has_invalid_packet) {
        _acu_state.last_time_pack_uv_fault_not_present = current_millis;
    }
    // Update temp fault time stamps
    if (input_state.max_board_temp < _parameters.charging_ot_thresh_c || has_invalid_packet) { // charging ot thresh will be the lower of the 2
        _acu_state.last_time_board_ot_fault_not_present = current_millis;
    }
    celsius cell_ot_thresh = _acu_state.charging_enabled ? _parameters.charging_ot_thresh_c : _parameters.running_ot_thresh_c;
    if (input_state.max_cell_temp < cell_ot_thresh || has_invalid_packet) {
        _acu_state.last_time_cell_ot_fault_not_present = current_millis;
    }
    if (input_state.global_invalid_packet_count > _parameters.invalid_packet_count_thresh) {
        _acu_state.last_time_invalid_packet_present = current_millis;
    }
    _acu_state.prev_time_stamp = current_millis;

    // Determine if there are any faults in the system : ov, uv, under pack voltage, board ot, cell ot ONLY if the data packet is all valid
    _acu_state.has_fault = _check_faults(current_millis);

    return _acu_state;
}

template <size_t num_cells>
std::array<bool, num_cells> ACUController<num_cells>::_calculate_cell_balance_statuses(std::array<volt, num_cells> voltages, volt min_voltage)
{
    std::array<bool, num_cells> cb = {false};

    for (size_t cell = 0; cell < num_cells; cell++)
    {
        volt cell_voltage = voltages[cell];
        if ((cell_voltage) - min_voltage > _parameters.v_diff_to_init_cb) // && max_voltage - (cell_voltage) < 200 &&
        {                                   
            cb[cell] = true;
        }
    }
    return cb;
}

template <size_t num_cells>
bool ACUController<num_cells>::_check_faults(time_ms current_millis)
{   
    return _check_voltage_faults(current_millis) || _check_temperature_faults(current_millis);
}

template <size_t num_cells>
bool ACUController<num_cells>::_check_voltage_faults(time_ms current_millis)
{
    bool ov_fault = (current_millis - _acu_state.last_time_ov_fault_not_present) > _parameters.max_allowed_voltage_fault_dur;
    bool uv_fault = (current_millis - _acu_state.last_time_uv_fault_not_present) > _parameters.max_allowed_voltage_fault_dur;
    bool pack_fault = (current_millis - _acu_state.last_time_pack_uv_fault_not_present) > _parameters.max_allowed_voltage_fault_dur;
    return ov_fault || uv_fault || pack_fault;
}

template <size_t num_cells>
bool ACUController<num_cells>::_check_temperature_faults(time_ms current_millis)
{
    bool cell_ot_fault = (current_millis - _acu_state.last_time_cell_ot_fault_not_present) > _parameters.max_allowed_temp_fault_dur;
    bool board_ot_fault = (current_millis - _acu_state.last_time_board_ot_fault_not_present) > _parameters.max_allowed_temp_fault_dur;
    return cell_ot_fault || board_ot_fault;
}

template <size_t num_cells>
bool ACUController<num_cells>::_check_invalid_packet_faults(time_ms current_millis)
{   
    bool invalid_packet_fault = (current_millis - _acu_state.last_time_invalid_packet_present) > _parameters.max_allowed_invalid_packet_fault_dur;
    return invalid_packet_fault;
}

template <size_t num_cells>
void ACUController<num_cells>::_coulomb_counting()
{
    // Numbers
}