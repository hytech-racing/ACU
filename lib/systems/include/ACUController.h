#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"
#include "SharedFirmwareTypes.h"

using time_ms = unsigned long;

namespace acu_controller_default_params
{
    constexpr const volt OV_THRESH = 4.2; // Volts
    constexpr const volt UV_THRESH = 3.2; // Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0; // Volts
    constexpr const celsius CHARGING_OT_THRESH = 60.0; // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 45.0; // Celsius
    constexpr const time_ms MAX_VOLTAGE_FAULT_DUR = 500; // At 4 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const time_ms MAX_TEMP_FAULT_DUR = 500; // Same as voltage fault count
};

template <size_t num_chips>
struct ACUControllerData_s
{
    time_ms uv_start_time;
    time_ms ov_start_time;
    time_ms cell_ot_start_time;
    time_ms board_ot_start_time;
    time_ms pack_uv_start_time;
    bool has_fault;
    bool charging_enabled;
    std::array<uint16_t, num_chips> cell_balance_statuses;
};

template<size_t num_chips>
class ACUController
{
public:

    using ACUStatus = ACUControllerData_s<num_chips>;

    /**
     * ACU Controller Constructor
     * @param ov_thresh_v over voltage threshold value | units in volts 
     * @param uv_thresh_v under voltage threshold value | units in volts
     * @param charging_ot_thresh_c overtemp threshold value | units in celsius
     * @param funning_ot_thresh_c overtemp threshold value | units in celsius
     * @param min_pack_total_voltage minimum pack total voltage | units in volts
     * @param max_volt_fault_dur max number of voltage faults allowed
     * @param max_temp_fault_dur max number of temp faults allowed
    */
    ACUController(volt ov_thresh_v = acu_controller_default_params::OV_THRESH, 
                    volt uv_thresh_v = acu_controller_default_params::UV_THRESH, 
                    celsius charging_ot_thresh_c = acu_controller_default_params::CHARGING_OT_THRESH, 
                    celsius running_ot_thresh_c = acu_controller_default_params::RUNNING_OT_THRESH,
                    volt min_pack_total_voltage = acu_controller_default_params::MIN_PACK_TOTAL_VOLTAGE,
                    time_ms max_volt_fault_dur = acu_controller_default_params::MAX_VOLTAGE_FAULT_DUR,
                    time_ms max_temp_fault_dur = acu_controller_default_params::MAX_TEMP_FAULT_DUR) : 
        _ov_thresh_v(ov_thresh_v),
        _uv_thresh_v(uv_thresh_v),
        _charging_ot_thresh_c(charging_ot_thresh_c),
        _running_ot_thresh_c(running_ot_thresh_c),
        _min_pack_total_v(min_pack_total_voltage),
        _max_allowed_voltage_fault_dur(max_volt_fault_dur),
        _max_allowed_temp_fault_dur(max_temp_fault_dur)
        {};

    /**
     * @brief Initialize the status time stamps because we don't want accidental sudden faults
     */
    void init(time_ms system_start_time);

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     */
    ACUStatus evaluate_accumulator(time_ms current_millis, std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, volt pack_voltage,
                                                volt min_voltage, volt max_voltage, celsius max_cell_temp, celsius max_board_temp);

private:
    /**
     * Calculate Cell Balancing values
     * @pre cell charging is enabled
     * @post _acu_state.cell_balance_statuses will have the new values
     */
    std::array<uint16_t, num_chips> _calculate_cell_balance_statuses(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, volt min_voltage);

    /**
     * @pre data has been gathered
     * @return boolean, true if there exists any fault
     */
    bool _check_faults(time_ms current_millis);

    /**
     * @pre voltage data has been gathered
     * @return boolean, true if there exists at least 1 voltage fault
     */
    bool _check_voltage_faults(time_ms current_millis);

    /**
     * @pre temperature data has been gathered
     * @return boolean, true if there exists a temperature fault
     */
    bool _check_temperature_faults(time_ms current_millis);

    /**
     * NOTE: TBD
    */
    void _coulomb_counting();

private: 
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
    */
    ACUStatus _acu_state = {};

    // Over-voltage threshold in volts
    const volt _ov_thresh_v = 0;

    // Under-voltage threshold in volts
    const volt _uv_thresh_v = 0;

    // Overtemp threshold in celsius
    const celsius _charging_ot_thresh_c = 0;

    // Overtemp threshold in celsius
    const celsius _running_ot_thresh_c = 0;

    // Minimum voltage threshold for the entire battery pack
    const volt _min_pack_total_v = 0;

    // Maximum number of voltage faults allowed before watchdog shuts off
    const time_ms _max_allowed_voltage_fault_dur = 0;

    // Maximum number of temp faults allowed before watchdog shuts off
    const time_ms _max_allowed_temp_fault_dur = 0;
};

#include "ACUController.tpp"
#endif