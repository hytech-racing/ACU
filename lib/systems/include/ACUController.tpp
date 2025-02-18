#include "ACUController.h"

template <size_t num_chips>
bool ACUController<num_chips>::_check_faults()
{
    return _check_voltage_faults() || _check_temperature_faults();
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_voltage_faults()
{
    return _acu_state.ov_counter > max_allowed_voltage_faults || _acu_state.uv_counter > max_allowed_voltage_faults;
}

template <size_t num_chips>
bool ACUController<num_chips>::_check_temperature_faults()
{
    return _acu_state.ot_counter > max_allowed_temp_faults;
}

template <size_t num_chips>
void ACUController<num_chips>::update_acu_state(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, float min_voltage, float max_voltage)
{
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
                if (_acu_state.charging_enabled) // We're only going to be balancing during charging
                {
                    if ((cell_voltage) - min_voltage > 200) // && max_voltage - (cell_voltage) < 200 && 
                    { // balance if the cell voltage differential from the max voltage is .02V or less and if the cell voltage differential from the minimum voltage is 0.02V or greater (progressive)
                        chip_balance_status = (0b1 << cell) | chip_balance_status;
                    }
                }
                // Check Faults
                // Check overvoltage
                if (cell_voltage > maximum_allowed_voltage)
                {
                    _acu_state.ov_counter++;
                }
                else
                {
                    _acu_state.ov_counter = 0;
                }
                // Check undervoltage
                if (cell_voltage < minimum_allowed_voltage)
                {
                    _acu_state.uv_counter++;
                }
                else
                {
                    _acu_state.uv_counter = 0;
                }
            }
        }
        // Assign cell 12 bit cell balance status to the chip index
        _acu_state.cell_balance_statuses[chip] = chip_balance_status;
    }
    _acu_state.has_voltage_fault = _check_faults();
}

template <size_t num_chips>
void ACUController<num_chips>::_columb_counting()
{
    // Numbers
}