#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"
#include "SharedFirmwareTypes.h"

namespace acu_controller_default_params
{
    constexpr const volt OV_THRESH = 4.2; // Volts
    constexpr const volt UV_THRESH = 3.2; // Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0; // Volts
    constexpr const celsius CHARGING_OT_THRESH = 60.0; // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 45.0; // Celsius
    constexpr const size_t MAX_NUM_VOLTAGE_FAULTS = 12; // At 4 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const size_t MAX_NUM_TEMP_FAULTS = 12; // Same as voltage fault count
};

template <size_t num_chips>
struct ACUControllerData_s
{
    size_t uv_counter;
    size_t ov_counter;
    size_t cell_ot_counter;
    size_t board_ot_counter;
    bool has_voltage_fault;
    bool charging_enabled;
    bool current_pulse;
    volt pack_total_voltage;
    std::array<uint16_t, num_chips> cell_balance_statuses;
};

template<size_t num_chips>
class ACUController
{
public:

    using ACUData = ACUControllerData_s<num_chips>;

    /**
     * ACU Controller Constructor
     * @param ov_thresh_v over voltage threshold value | units in volts 
     * @param uv_thresh_v under voltage threshold value | units in volts
     * @param charging_ot_thresh_c overtemp threshold value | units in celsius
     * @param funning_ot_thresh_c overtemp threshold value | units in celsius
     * @param min_pack_total_voltage minimum pack total voltage | units in volts
     * @param max_volt_faults max number of voltage faults allowed
     * @param max_temp_faults max number of temp faults allowed
    */
    ACUController(volt ov_thresh_v = acu_controller_default_params::OV_THRESH, 
                    volt uv_thresh_v = acu_controller_default_params::UV_THRESH, 
                    celsius charging_ot_thresh_c = acu_controller_default_params::CHARGING_OT_THRESH, 
                    celsius running_ot_thresh_c = acu_controller_default_params::RUNNING_OT_THRESH,
                    volt min_pack_total_voltage = acu_controller_default_params::MIN_PACK_TOTAL_VOLTAGE,
                    size_t max_volt_faults = acu_controller_default_params::MAX_NUM_VOLTAGE_FAULTS,
                    size_t max_temp_faults = acu_controller_default_params::MAX_NUM_TEMP_FAULTS) : 
        _ov_thresh_v(ov_thresh_v),
        _uv_thresh_v(uv_thresh_v),
        _charging_ot_thresh_c(charging_ot_thresh_c),
        _running_ot_thresh_c(running_ot_thresh_c),
        _min_pack_total_v(min_pack_total_voltage),
        _max_allowed_voltage_faults(max_volt_faults),
        _max_allowed_temp_faults(max_temp_faults)
        {};

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     */
    void update_acu_state(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, 
        std::array<celsius, 4 * num_chips> cell_temps, std::array<celsius, num_chips> board_temps, float min_voltage, float max_voltage);

    /**
     * cell balance status getter function
     * @post provides access to the ACU Controller's state's cell balance status array of length num chips
     * With the intention of feeding the cell balance statuses into the BMS Driver to rewrite configurations
    */
    std::array<uint16_t, num_chips> get_cell_balance_params() {
        return _acu_state.cell_balance_statuses;
    }

private:
    /**
     * @pre data has been gathered
     * @return boolean, true if there exists any fault
     */
    bool _check_faults();

    /**
     * @pre voltage data has been gathered
     * @return boolean, true if there exists at least 1 voltage fault
     */
    bool _check_voltage_faults();

    /**
     * @pre temperature data has been gathered
     * @return boolean, true if there exists a temperature fault
     */
    bool _check_temperature_faults();

    /**
     * NOTE: TBD
    */
    void _coulumb_counting();

private: 
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
    */
    ACUData _acu_state = {};

    // Overvoltage threshold in volts
    const volt _ov_thresh_v = 0;

    // Undervoltage threshold in volts
    const volt _uv_thresh_v = 0;

    // Overtemp threshold in celcius
    const celsius _charging_ot_thresh_c = 0;

    // Overtemp threshold in celcius
    const celsius _running_ot_thresh_c = 0;

    // Minimum voltage threshold for the entire battery pack
    const volt _min_pack_total_v = 0;

    // Maximum number of voltage faults allowed before watchdog shuts off
    const size_t _max_allowed_voltage_faults = 0;

    // Maximum number of temp faults allowed before watchdog shuts off
    const celsius _max_allowed_temp_faults = 0;
};

#include "ACUController.tpp"
#endif