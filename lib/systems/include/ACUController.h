#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"
#include "etl/singleton.h"
#include "SharedFirmwareTypes.h"

// Won't actually be here, but I need the struct to be in SharedFirmwareTypes.h once this gets checked off, then i'll remove this
#include "ACU_Globals.h"

using time_ms = uint32_t;

namespace acu_controller_default_params
{
    constexpr const volt OV_THRESH = 4.2; // Volts
    constexpr const volt UV_THRESH = 3.05; // Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0; // Volts
    constexpr const celsius CHARGING_OT_THRESH = 45.0; // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 60.0; // Celsius
    constexpr const time_ms MAX_VOLTAGE_FAULT_DUR = 500; // At 4 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const time_ms MAX_TEMP_FAULT_DUR = 500; // Same as voltage fault count
    constexpr const volt VOLTAGE_DIFF_TO_INIT_CB = 0.02; // differential with lowest cell voltage to enable cell balancing for a cell
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

struct ACUControllerParameters {
    volt ov_thresh_v = 0;
    volt uv_thresh_v = 0;
    celsius charging_ot_thresh_c = 0;
    celsius running_ot_thresh_c = 0;
    volt min_pack_total_v = 0;
    time_ms max_allowed_voltage_fault_dur = 0;
    time_ms max_allowed_temp_fault_dur = 0;
    volt v_diff_to_init_cb = 0;
};

template<size_t num_cells>
class ACUController
{
public:

    using ACUStatus = ACUControllerData_s<num_cells>;

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
                    time_ms max_temp_fault_dur = acu_controller_default_params::MAX_TEMP_FAULT_DUR,
                    volt v_diff_init_cb = acu_controller_default_params::VOLTAGE_DIFF_TO_INIT_CB) : 
        _parameters {
            ov_thresh_v,
            uv_thresh_v,
            charging_ot_thresh_c,
            running_ot_thresh_c,
            min_pack_total_voltage,
            max_volt_fault_dur,
            max_temp_fault_dur,
            v_diff_init_cb
        }
        {};

    /**
     * @brief Initialize the status time stamps because we don't want accidental sudden faults
     */
    void init(time_ms system_start_time);

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     */
    ACUStatus evaluate_accumulator(time_ms current_millis, bool charging_enabled, const ACUData_s<num_cells> &input_state);
private:
    /**
     * Calculate Cell Balancing values
     * @pre cell charging is enabled
     * @post _acu_state.cell_balance_statuses will have the new values
     */
    std::array<bool, num_cells> _calculate_cell_balance_statuses(std::array<volt, num_cells> voltages, volt min_voltage);

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

public: // private:
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
    */
    ACUStatus _acu_state = {};

    /**
     * @brief ACU Controller Parameters holder
    */
    const ACUControllerParameters _parameters = {};
};

template<size_t num_cells>
using ACUControllerInstance = etl::singleton<ACUController<num_cells>>;

#include "ACUController.tpp"
#endif