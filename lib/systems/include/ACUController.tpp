#include "ACUController.h"

template <size_t num_chips>
void ACUController<num_chips>::update_acu_state(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, 
                                                std::array<celsius, 4 * num_chips> cell_temps,
                                                std::array<celsius, num_chips> board_temps, float min_voltage, float max_voltage)
{
    
    // Cell balancing and voltage fault checking
    volt pack_total_voltage = 0;

    bool ov_fault_triggered = false;
    bool uv_fault_triggered = false;

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
                // Add to pack total calculation
                pack_total_voltage += cell_voltage;

                // We're only going to be balancing during charging
                if (_acu_state.charging_enabled)
                {
                    if ((cell_voltage) - min_voltage > 200) // && max_voltage - (cell_voltage) < 200 && 
                    { // balance if the cell voltage differential from the max voltage is .02V or less and if the cell voltage differential from the minimum voltage is 0.02V or greater (progressive)
                        chip_balance_status = (0b1 << cell) | chip_balance_status;
                    }
                }
                // Check Faults
                // Check cell over voltage
                if (cell_voltage > _ov_thresh_v)
                {
                    ov_fault_triggered = true;
                    // We want to check and see if there is a voltage fault in the system after we measure all of the cell voltages
                    // so as to not accidentally find a normal cell at the end and set counter to 0
                    // so reset the VOLTAGE counters to 0 after ALL of the parsing is done
                }
                // Check cell under voltage
                if (cell_voltage < _uv_thresh_v)
                {   
                    uv_fault_triggered = true;
                }
            }
        }
        // Assign cell 12 bit cell balance status to the chip index
        _acu_state.cell_balance_statuses[chip] = chip_balance_status;
    }
    // Assign pack total voltage to acu state, accessible by _check_faults()
    _acu_state.pack_total_voltage = pack_total_voltage;

    // Update voltage fault counters
    _acu_state.ov_counter = ov_fault_triggered ? _acu_state.ov_counter + 1 : 0;
    _acu_state.uv_counter = uv_fault_triggered ? _acu_state.uv_counter + 1 : 0;

    // Temperature fault checking
    // Cell Temperatures 
    celsius max_temp = _acu_state.charging_enabled ? _charging_ot_thresh_c : _running_ot_thresh_c;

    bool cell_ot_fault_triggered = false;
    bool board_ot_fault_triggered = false;

    for (auto cell_temp : cell_temps) // So the way this is implemented, the ot_counter increments per read_data call IF there is a fault
    {
        if (cell_temp > max_temp)
        {
            cell_ot_fault_triggered = true;
        }
    }
    // Board Temperatures
    for (auto board_temp : board_temps)
    {
        if (board_temp > _charging_ot_thresh_c) // Choose charging ot threshold because it's always going to be lower, and we don't expect the board to be hotter than the cells
        {
            board_ot_fault_triggered = true;
        }
    }

    // Update temperature fault counters
    _acu_state.cell_ot_counter = cell_ot_fault_triggered ? _acu_state.cell_ot_counter + 1 : 0;
    _acu_state.board_ot_counter = board_ot_fault_triggered ? _acu_state.board_ot_counter + 1 : 0;

    // Determine if there are any faults in the system : ov, uv, under pack voltage, board ot, cell ot
    _acu_state.has_voltage_fault = _check_faults();

}

template <size_t num_chips>
bool ACUController<num_chips>::_check_faults()
{
    return _check_voltage_faults() || _check_temperature_faults();
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_voltage_faults()
{
    bool ov_fault = _acu_state.ov_counter > _max_allowed_voltage_faults;
    bool uv_fault = _acu_state.uv_counter > _max_allowed_voltage_faults;
    bool pack_fault = _acu_state.pack_total_voltage < _min_pack_total_v;
    return ov_fault || uv_fault || pack_fault;
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_temperature_faults()
{
    bool cell_ot_fault = _acu_state.cell_ot_counter > _max_allowed_temp_faults;
    bool board_ot_fault = _acu_state.board_ot_counter > _max_allowed_temp_faults;
    return cell_ot_fault > board_ot_fault;
}

template <size_t num_chips>
void ACUController<num_chips>::_coulumb_counting()
{
    // Numbers
}