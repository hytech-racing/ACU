#include "ACUController.h"

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
void ACUController<num_cells, num_celltemps, num_boardtemps>::init(time_ms system_start_time, volt pack_voltage)
{
    _acu_state.last_time_ov_fault_not_present = system_start_time;
    _acu_state.last_time_uv_fault_not_present = system_start_time;
    _acu_state.last_time_board_ot_fault_not_present = system_start_time;
    _acu_state.last_time_cell_ot_fault_not_present = system_start_time;
    _acu_state.last_time_pack_uv_fault_not_present = system_start_time;
    _acu_state.last_time_invalid_packet_present = system_start_time;
    _acu_state.prev_bms_time_stamp = system_start_time;
    _acu_state.SoC = (pack_voltage <= _acu_parameters.pack_specs.pack_min_voltage) ? 0.0 : ((pack_voltage - _acu_parameters.pack_specs.pack_min_voltage) / (_acu_parameters.pack_specs.pack_max_voltage - _acu_parameters.pack_specs.pack_min_voltage));
    _acu_state.balancing_enabled = false;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
typename ACUController<num_cells, num_celltemps, num_boardtemps>::ACUStatus
ACUController<num_cells, num_celltemps, num_boardtemps>::evaluate_accumulator(time_ms current_millis, const ACUData_s<num_cells, num_celltemps, num_boardtemps> &input_state, float pack_current)
{
    _acu_state.charging_enabled = input_state.charging_enabled;

    bool has_invalid_packet = false;
    if (input_state.max_consecutive_invalid_packet_count != 0)
    { // meaning that at least one of the packets is invalid
        has_invalid_packet = true;
    }

    // Cell balancing calculations
    bool previously_balancing = _acu_state.balancing_enabled;

    bool balance_enableable = ((previously_balancing && (input_state.max_board_temp < _acu_parameters.thresholds.balance_temp_limit_c)) ||
                               (!previously_balancing && (input_state.max_board_temp < _acu_parameters.thresholds.balance_enable_temp_c)));

    bool allow_balancing = ((balance_enableable && _acu_state.charging_enabled));

    if (allow_balancing)
    {
        _acu_state.balancing_enabled = true;
        _acu_state.cell_balancing_statuses = _calculate_cell_balance_statuses(input_state.voltages, input_state.min_cell_voltage);
    }
    else
    { // Fill with zeros, no balancing
        _acu_state.balancing_enabled = false;
        _acu_state.cell_balancing_statuses.fill(0);
    }

    // Update voltage fault time stamps with IR compensation
    // Internal_V = Read_V + (IR × discharge_current), where discharge_current is positive during discharge
    const float discharge_current = -pack_current; // Positive during discharge, negative during charge

    // OV check with IR compensation (main concern during charging and recharge)
    volt max_cell_voltage_to_check = input_state.max_cell_voltage;
    if (input_state.max_cell_voltage >= _acu_parameters.thresholds.cell_overvoltage_thresh_v)
    {
        // Only calculate IR compensation when approaching OV threshold
        max_cell_voltage_to_check = input_state.max_cell_voltage + (CELL_IR * discharge_current);
    }
    if (max_cell_voltage_to_check < _acu_parameters.thresholds.cell_overvoltage_thresh_v || has_invalid_packet)
    {
        _acu_state.last_time_ov_fault_not_present = current_millis;
    }

    // UV check with IR compensation (main concern during discharging)
    volt min_cell_voltage_to_check = input_state.min_cell_voltage;
    if (input_state.min_cell_voltage <= _acu_parameters.thresholds.cell_undervoltage_thresh_v)
    {
        // Only calculate IR compensation when approaching UV threshold
        min_cell_voltage_to_check = input_state.min_cell_voltage + (CELL_IR * discharge_current);
    }
    if (min_cell_voltage_to_check > _acu_parameters.thresholds.cell_undervoltage_thresh_v || has_invalid_packet)
    {
        _acu_state.last_time_uv_fault_not_present = current_millis;
    }
    if (input_state.pack_voltage > _acu_parameters.thresholds.min_pack_total_v || has_invalid_packet)
    {
        _acu_state.last_time_pack_uv_fault_not_present = current_millis;
    }
    // Update temp fault time stamps
    celsius ot_thresh = _acu_state.charging_enabled ? _acu_parameters.thresholds.charging_ot_thresh_c : _acu_parameters.thresholds.running_ot_thresh_c;
    if (input_state.max_board_temp < ot_thresh || has_invalid_packet)
    { // charging ot thresh will be the lower of the 2
        _acu_state.last_time_board_ot_fault_not_present = current_millis;
    }
    if (input_state.max_cell_temp < ot_thresh || has_invalid_packet)
    {
        _acu_state.last_time_cell_ot_fault_not_present = current_millis;
    }
    if (input_state.max_consecutive_invalid_packet_count < _acu_parameters.invalid_packet_count_thresh)
    {
        _acu_state.last_time_invalid_packet_present = current_millis;
    }
    _acu_state.prev_bms_time_stamp = current_millis;

    // Determine if there are any faults in the system : ov, uv, under pack voltage, board ot, cell ot ONLY if the data packet is all valid
    _acu_state.has_fault = _check_faults(current_millis);

    return _acu_state;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
std::array<bool, num_cells> ACUController<num_cells, num_celltemps, num_boardtemps>::_calculate_cell_balance_statuses(std::array<volt, num_cells> voltages, volt min_voltage)
{
    std::array<bool, num_cells> cb = {false};
    const volt min_discharge_voltage_thresh = 3.8F;
    for (size_t cell = 0; cell < num_cells; cell++)
    {
        volt cell_voltage = voltages[cell];
        if (((cell_voltage)-min_voltage > _acu_parameters.thresholds.v_diff_to_init_cb) && (cell_voltage > min_discharge_voltage_thresh)) // && max_voltage - (cell_voltage) < 200 &&
        {
            cb[cell] = true;
        }
    }
    return cb;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
float ACUController<num_cells, num_celltemps, num_boardtemps>::get_state_of_charge(float em_current, uint32_t delta_time_ms)
{
    float delta_ah = (em_current) * ((float)(delta_time_ms / 1000.0f) / 3600.0f);  // amp hours
    _acu_state.SoC += delta_ah / _acu_parameters.pack_specs.pack_nominal_capacity; // should be -= but EM inverted
    if (_acu_state.SoC < 0.0)
        _acu_state.SoC = 0;
    if (_acu_state.SoC > 1.0)
        _acu_state.SoC = 1;
    return _acu_state.SoC;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
bool ACUController<num_cells, num_celltemps, num_boardtemps>::_check_faults(time_ms current_millis)
{
    return _check_voltage_faults(current_millis) || _check_temperature_faults(current_millis) || _check_invalid_packet_faults(current_millis);
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
bool ACUController<num_cells, num_celltemps, num_boardtemps>::_check_voltage_faults(time_ms current_millis)
{
    bool ov_fault = (current_millis - _acu_state.last_time_ov_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    bool uv_fault = (current_millis - _acu_state.last_time_uv_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    bool pack_fault = (current_millis - _acu_state.last_time_pack_uv_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    return ov_fault || uv_fault || pack_fault;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
bool ACUController<num_cells, num_celltemps, num_boardtemps>::_check_temperature_faults(time_ms current_millis)
{
    bool cell_ot_fault = (current_millis - _acu_state.last_time_cell_ot_fault_not_present) > _acu_parameters.fault_durations.max_allowed_temp_fault_dur;
    bool board_ot_fault = (current_millis - _acu_state.last_time_board_ot_fault_not_present) > _acu_parameters.fault_durations.max_allowed_temp_fault_dur;
    return cell_ot_fault || board_ot_fault;
}

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
bool ACUController<num_cells, num_celltemps, num_boardtemps>::_check_invalid_packet_faults(time_ms current_millis)
{
    bool invalid_packet_fault = (current_millis - _acu_state.last_time_invalid_packet_present) > _acu_parameters.fault_durations.max_allowed_invalid_packet_fault_dur;
    return invalid_packet_fault;
}