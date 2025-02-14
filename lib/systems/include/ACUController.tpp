#include "ACUController.h"

template<size_t num_chips>
bool check_faults(ACU_State_s<num_chips> acu_state) {
    return check_voltage_faults(acu_state.ov_counter, acu_state.uv_counter) || check_temperature_faults(acu_state.ot_counter);
}

bool check_voltage_faults(size_t ov_counter, size_t uv_counter) {
    #ifdef DEBUG
        Serial.print("# of Over Voltage Faults: "); Serial.println(ov_counter);
        Serial.print("# of Under Voltage Faults: "); Serial.println(uv_counter);
    #endif 

    return ov_counter > max_allowed_voltage_faults || uv_counter > max_allowed_voltage_faults;
}

bool check_temperature_faults(size_t ot_counter) {
    return ot_counter > max_allowed_temp_faults;
}

template <size_t num_chips>
void update_acu_state(ACU_State_s<num_chips> &acu_state, std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, float min_voltage, float max_voltage)
{
    for (size_t chip = 0; chip < num_chips; chip++)
    {
        uint16_t chip_balance_status;
        for (size_t cell = 0; cell < voltages[chip].size(); cell++)
        {   
            // Will only get voltage if not a null pointer
            if(voltages[chip][cell])
            {
                // Get cell voltage from optional
                volt cell_voltage = *voltages[chip][cell];
                if (max_voltage - (cell_voltage) < 200 && (cell_voltage) - min_voltage > 200)
                { // balance if the cell voltage differential from the max voltage is .02V or less and if the cell voltage differential from the minimum voltage is 0.02V or greater (progressive)
                    chip_balance_status = (0b1 << cell) | chip_balance_status;
                }
                // Check for faults
                if (cell_voltage > maximum_allowed_voltage) {
                    acu_state.ov_counter++;
                } else {
                    acu_state.ov_counter = 0;
                }
                if (cell_voltage < minimum_allowed_voltage) {
                    acu_state.uv_counter++;
                } else {
                    acu_state.uv_counter = 0;
                }
    
            }
            
        }
        acu_state.cell_balance_statuses[chip] = chip_balance_status;
    }
    acu_state.has_voltage_fault = check_faults(acu_state);
}

void columb_counting() {

}