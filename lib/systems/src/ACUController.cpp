#include "ACUController.h"


void ACUController::init(time_ms system_start_time, volt pack_voltage)
{
    _acu_state.last_time_ov_fault_not_present = system_start_time;
    _acu_state.last_time_uv_fault_not_present = system_start_time;
    _acu_state.last_time_board_ot_fault_not_present = system_start_time;
    _acu_state.last_time_cell_ot_fault_not_present = system_start_time;
    _acu_state.last_time_pack_uv_fault_not_present = system_start_time;
    _acu_state.last_time_invalid_packet_present = system_start_time;
    _acu_state.prev_bms_time_stamp = system_start_time;
    _acu_state.SoC = -1;
    _acu_state.lifetime_ah_throughput = 0.0f;
    _acu_state.SoH = 1.0f;
    _acu_state.SoE_percentage = 0.0f;
    _acu_state.remaining_pack_wh = 0.0f;
    _acu_state.balancing_enabled = false;
    _acu_state.high_side_contactor_welded = false;
    _acu_state.low_side_contactor_welded = false;
    _acu_state.bms_ok = true;
}

void ACUController::restore_lifetime_throughput(double restored_ah)
{
    // Restore the persisted accumulator and recompute SoH so it is valid before the first tick.
    _acu_state.lifetime_ah_throughput = fmax(0.0, restored_ah);
    _acu_state.SoH = compute_soh_from_throughput(_acu_state.lifetime_ah_throughput);
}

ACUControllerData_s ACUController::evaluate_accumulator(time_ms current_millis, const BMSCoreData_s &input_state, size_t max_consecutive_invalid_packet_count, float em_current, size_t num_of_voltage_cells, bool voltage_is_fresh)
{
    // _acu_state.charging_enabled = input_state.charging_enabled;

    bool has_invalid_packet = false;
    if (max_consecutive_invalid_packet_count != 0)
    { // meaning that at least one of the packets is invalid
        has_invalid_packet = true;
    }

    volt min_cell_voltage = input_state.min_cell_voltage;
    _acu_state.SoC = get_state_of_charge(em_current, current_millis - _acu_state.prev_bms_time_stamp, min_cell_voltage, current_millis, voltage_is_fresh);
    _acu_state.V1 = _soc_ekf.get_voltage();

    // State of Health via Ah throughput model
    float dt = static_cast<float>(current_millis - _acu_state.prev_bms_time_stamp) / _ms_to_seconds;
    double ah_step = (fabs(static_cast<double>(em_current)) * static_cast<double>(dt)) / _seconds_per_hour;
    _acu_state.lifetime_ah_throughput += ah_step;

    _acu_state.SoH = compute_soh_from_throughput(_acu_state.lifetime_ah_throughput);

    // State of Energy mapped from EKF SoC estimate
    _acu_state.SoE_percentage = _soc_ekf.get_soe_percentage();
    _acu_state.remaining_pack_wh = (_acu_state.SoE_percentage / 100.0f) * (soc_ekf_constants::TOTAL_USABLE_WH_PER_CELL_3C * _acu_state.SoH) * NUM_CELLS;

    // Cell balancing calculations
    bool previously_balancing = _acu_state.balancing_enabled;

    bool balance_enableable = (
        (previously_balancing && (input_state.max_board_temp < _acu_parameters.thresholds.balance_temp_limit_c)) ||
        (!previously_balancing && (input_state.max_board_temp < _acu_parameters.thresholds.balance_enable_temp_c))
    );

    bool allow_balancing = (balance_enableable && _acu_state.charging_enabled);

    if (allow_balancing)
    {
        _acu_state.balancing_enabled = true;
    }
    else
    {
        _acu_state.balancing_enabled = false;
    }

    // Update voltage fault time stamps with IR compensation
    // Internal_V = Read_V + (IR × discharge_current), where discharge_current is positive during discharge
    const float discharge_current = -em_current; // Positive during discharge, negative during charge

    // OV check with IR compensation (main concern during charging and recharge)
    volt internal_resistance_max_cell_voltage = input_state.max_cell_voltage;
    if (input_state.max_cell_voltage >= _acu_parameters.thresholds.cell_overvoltage_thresh_v)
    {
        // Only calculate IR compensation when approaching OV threshold
        internal_resistance_max_cell_voltage = input_state.max_cell_voltage + (_acu_parameters.pack_specs.pack_internal_resistance / static_cast<float>(num_of_voltage_cells) * discharge_current);
    }
    if (internal_resistance_max_cell_voltage < _acu_parameters.thresholds.cell_overvoltage_thresh_v || has_invalid_packet)
    {
        _acu_state.last_time_ov_fault_not_present = current_millis;
    }

    // UV check with IR compensation (main concern during discharging)
    volt min_cell_voltage_to_check = input_state.min_cell_voltage;
    if (input_state.min_cell_voltage <= _acu_parameters.thresholds.cell_undervoltage_thresh_v)
    {
        // Only calculate IR compensation when approaching UV threshold
        min_cell_voltage_to_check = input_state.min_cell_voltage + (_acu_parameters.pack_specs.pack_internal_resistance / static_cast<float>(num_of_voltage_cells) * discharge_current);
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
    if (max_consecutive_invalid_packet_count < _acu_parameters.invalid_packet_count_thresh)
    {
        _acu_state.last_time_invalid_packet_present = current_millis;
    }
    _acu_state.prev_bms_time_stamp = current_millis;

    // Determine if there are any faults in the system : ov, uv, under pack voltage, board ot, cell ot ONLY if the data packet is all valid
    _acu_state.has_fault = _check_faults(current_millis);

    // Determine if bms is ok
    _acu_state.bms_ok = _is_bms_ok(current_millis);

    return _acu_state;
}

void ACUController::calculate_cell_balance_statuses(bool* output, const volt* voltages, size_t num_of_voltage_cells, volt min_voltage)
{
    for (size_t cell = 0; cell < num_of_voltage_cells; cell++)
    {
        volt cell_voltage = voltages[cell]; //NOLINT
        if (((cell_voltage - min_voltage) > _acu_parameters.thresholds.v_diff_to_init_cb) && (cell_voltage > _acu_parameters.thresholds.min_discharge_voltage_thresh))
        {
            output[cell] = true; //NOLINT
        } else
        {
            output[cell] = false; //NOLINT
        }
    }
}

float ACUController::_get_soc_from_voltage(volt min_cell_voltage)
{
    static constexpr size_t table_size = 101;

    if (min_cell_voltage >= SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[0])
    {
        return 1.0f;
    }
    if (min_cell_voltage <= SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[table_size - 1])
    {
        return 0.0f;
    }

    for (size_t i = 0; i < table_size - 1; i++)
    {
        if (min_cell_voltage <= SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[i] && min_cell_voltage > SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[i + 1]) //NOLINT
        {
            float v_high = SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[i]; //NOLINT
            float v_low = SOCKalmanFilter::VOLTAGE_LOOKUP_TABLE[i + 1]; //NOLINT
            float soc_high = (float)(table_size - 1 - i) / (table_size - 1);
            float soc_low = (float)(table_size - 1 - (i + 1)) / (table_size - 1);

            return soc_low + (min_cell_voltage - v_low) / (v_high - v_low) * (soc_high - soc_low);
        }
    }

    return 0.0f;
}

float ACUController::get_state_of_charge(float em_current, uint32_t delta_time_ms, volt min_cell_voltage, time_ms current_millis, bool voltage_is_fresh)
{
    if (!_ekf_initialized)
    {
        if (!voltage_is_fresh)
        {
            return _acu_state.SoC;
        }
        if (min_cell_voltage < acu_controller_default_parameters::MIN_CELL_VOLTAGE_FOR_SOC)
        {
            return 0.0f;
        }
        else
        {
            _soc_ekf.init(min_cell_voltage);
            _ekf_initialized = true;
            _acu_state.SoC = _soc_ekf.get_soc();
            return _acu_state.SoC;
        }
    }

    float dt = static_cast<float>(delta_time_ms) / _ms_to_seconds; // in seconds

    // we will use coulomb counting for the normal implementation of getting state of charge
    // whenever the car has been at rest (em voltage and em current at 0) for 30 mins, then we can correct the SoC to the voltage look up table value
    // we will reset the soc with the voltage look up value
    // we want to then start coulomb counting from this point, we also want to restart a 30 min timer, so we can set the start time to now

    bool is_stabilized = (fabs(em_current) <= STABILIZED_CURRENT_THRESH);
    if (is_stabilized)
    {
        if (_acu_state.first_zero_current_time_stamp == 0)
        {
            _acu_state.first_zero_current_time_stamp = current_millis;
        }
        // we have another 0 current, so we need to see if we have rested for long enough
        if ((current_millis - _acu_state.first_zero_current_time_stamp) >= MIN_STABILIZED_CURRENT_DURATION_MS)
        {
            if (voltage_is_fresh)
            {
                _acu_state.SoC = _get_soc_from_voltage(min_cell_voltage);
                _soc_ekf.reset_soc(_acu_state.SoC);

                return _acu_state.SoC;
            }
        }
    }
    else
    {
        _acu_state.first_zero_current_time_stamp = 0;
    }

    EKFState_s ekf_state = _soc_ekf.update(em_current, min_cell_voltage, dt, voltage_is_fresh, _acu_state.SoH);
    _acu_state.SoC = ekf_state.soc;

    return _acu_state.SoC;
}


bool ACUController::_is_bms_ok(time_ms current_millis)
{
    if (_acu_state.has_fault)
    {
        _acu_state.last_bms_not_ok_eval = current_millis;
        return false;
    }
    else if (!_acu_state.bms_ok && (current_millis - _acu_state.last_bms_not_ok_eval > _bms_not_ok_hold_time_ms))
    {
       return true;
    }

    return _acu_state.bms_ok;
}


bool ACUController::_check_faults(time_ms current_millis)
{
    return _check_voltage_faults(current_millis) || _check_temperature_faults(current_millis) || _check_invalid_packet_faults(current_millis);
}


bool ACUController::_check_voltage_faults(time_ms current_millis)
{
    bool ov_fault = (current_millis - _acu_state.last_time_ov_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    bool uv_fault = (current_millis - _acu_state.last_time_uv_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    bool pack_fault = (current_millis - _acu_state.last_time_pack_uv_fault_not_present) > _acu_parameters.fault_durations.max_allowed_voltage_fault_dur;
    return ov_fault || uv_fault || pack_fault;
}


bool ACUController::_check_temperature_faults(time_ms current_millis)
{
    bool cell_ot_fault = (current_millis - _acu_state.last_time_cell_ot_fault_not_present) > _acu_parameters.fault_durations.max_allowed_temp_fault_dur;
    bool board_ot_fault = (current_millis - _acu_state.last_time_board_ot_fault_not_present) > _acu_parameters.fault_durations.max_allowed_temp_fault_dur;
    return cell_ot_fault || board_ot_fault;
}


bool ACUController::_check_invalid_packet_faults(time_ms current_millis)
{
    bool invalid_packet_fault = (current_millis - _acu_state.last_time_invalid_packet_present) > _acu_parameters.fault_durations.max_allowed_invalid_packet_fault_dur;
    return invalid_packet_fault;
}

bool ACUController::check_is_contactor_welded(volt pack_voltage_adc, volt ts_voltage_adc)
{
    _acu_state.low_side_contactor_welded = pack_voltage_adc > _acu_parameters.thresholds.ts_isolation_voltage;
    _acu_state.high_side_contactor_welded = ts_voltage_adc > _acu_parameters.thresholds.ts_isolation_voltage;

    bool is_welded = _acu_state.low_side_contactor_welded || _acu_state.high_side_contactor_welded;
    return is_welded;
}