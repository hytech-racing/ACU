#include "ACUController.h"

template <size_t num_chips>
typename ACUController<num_chips>::ACUStatus
ACUController<num_chips>::evaluate_accumulator(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, volt pack_voltage,
                                                volt min_voltage, volt max_voltage, celsius max_cell_temp, celsius max_board_temp)
{

    // Cell balancing calculations
    if (_acu_state.charging_enabled)
    {
        _acu_state.cell_balance_statuses = _calculate_cell_balance_statuses(voltages, min_voltage);
    } else {
        // Fill with zeros, no balancing
        _acu_state.cell_balance_statuses.fill(0);
    }

    // Update voltage fault counters
    // _acu_state.ov_counter = ov_fault_triggered ? _acu_state.ov_counter + 1 : 0;
    // _acu_state.uv_counter = uv_fault_triggered ? _acu_state.uv_counter + 1 : 0;

    // Temperature fault checking
    // Cell Temperatures
    celsius max_temp = _acu_state.charging_enabled ? _charging_ot_thresh_c : _running_ot_thresh_c;

    bool cell_ot_fault_triggered = false;
    bool board_ot_fault_triggered = false;

    // for (auto cell_temp : cell_temps) // So the way this is implemented, the ot_counter increments per read_data call IF there is a fault
    // {
    //     if (cell_temp > max_temp)
    //     {
    //         cell_ot_fault_triggered = true;
    //         break;
    //     }
    // }
    // // Board Temperatures
    // for (auto board_temp : board_temps)
    // {
    //     if (board_temp > _charging_ot_thresh_c) // Choose charging ot threshold because it's always going to be lower, and we don't expect the board to be hotter than the cells
    //     {
    //         board_ot_fault_triggered = true;
    //         break;
    //     }
    // }

    // Update temperature fault counters
    _acu_state.cell_ot_counter = cell_ot_fault_triggered ? _acu_state.cell_ot_counter + 1 : 0;
    _acu_state.board_ot_counter = board_ot_fault_triggered ? _acu_state.board_ot_counter + 1 : 0;

    // Determine if there are any faults in the system : ov, uv, under pack voltage, board ot, cell ot
    _acu_state.has_voltage_fault = _check_faults();
}

template <size_t num_chips>
std::array<uint16_t, num_chips> ACUController<num_chips>::_calculate_cell_balance_statuses(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, volt min_voltage)
{
    std::array<uint16_t, num_chips> cb;

    for (size_t chip = 0; chip < num_chips; chip++)
    {
        uint16_t chip_balance_status = 0;
        for (size_t cell = 0; cell < voltages[chip].size(); cell++)
        {
            // Will only get voltage if not a null pointer
            if (voltages[chip][cell])
            {
                // Get cell voltage from optional
                volt cell_voltage = *voltages[chip][cell];
                if ((cell_voltage) - min_voltage > 200) // && max_voltage - (cell_voltage) < 200 &&
                {                                     // balance if the cell voltage differential from the max voltage is .02V or less and if the cell voltage differential from the minimum voltage is 0.02V or greater (progressive)
                    chip_balance_status = (0b1 << cell) | chip_balance_status;
                }
            }
        }
        cb[chip] = chip_balance_status;
    }
    return cb;
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_faults()
{
    return _check_voltage_faults() || _check_temperature_faults();
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_voltage_faults()
{
    // bool ov_fault = _acu_state.ov_counter > _max_allowed_voltage_faults;
    // bool uv_fault = _acu_state.uv_counter > _max_allowed_voltage_faults;
    bool pack_fault = _acu_state.pack_total_voltage < _min_pack_total_v;
    return false; // ov_fault || uv_fault || pack_fault;
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_temperature_faults()
{
    // bool cell_ot_fault = _acu_state.cell_ot_counter > _max_allowed_temp_faults;
    // bool board_ot_fault = _acu_state.board_ot_counter > _max_allowed_temp_faults;
    return false; // cell_ot_fault > board_ot_fault;
}

template <size_t num_chips>
void ACUController<num_chips>::_coulomb_counting()
{
    // Numbers
}