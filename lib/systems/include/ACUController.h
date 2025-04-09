#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/singleton.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

using time_ms = uint32_t;

namespace acu_controller_default_params
{
    constexpr const volt OV_THRESH = 4.2; // Volts
    constexpr const volt UV_THRESH = 3.05; // Volts
    constexpr const volt MIN_PACK_TOTAL_VOLTAGE = 420.0; // Volts
    constexpr const celsius CHARGING_OT_THRESH = 45.0; // Celsius
    constexpr const celsius RUNNING_OT_THRESH = 60.0; // Celsius
    constexpr const size_t MAX_INVALID_PACKET_FAULT_COUNT = 1000000; // Same as voltage fault count
    constexpr const time_ms MAX_VOLTAGE_FAULT_DUR = 500; // At 4 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const time_ms MAX_TEMP_FAULT_DUR = 500; // Same as voltage fault count
    constexpr const time_ms MAX_INVALID_PACKET_FAULT_DUR = 500; // In cases in EMI, we will need more leniency with invalid packet faults
    constexpr const volt VOLTAGE_DIFF_TO_INIT_CB = 0.02; // differential with lowest cell voltage to enable cell balancing for a cell
};

template <size_t num_cells>
struct ACUControllerData_s
{
    time_ms last_time_uv_fault_not_present;
    time_ms last_time_ov_fault_not_present;
    time_ms last_time_cell_ot_fault_not_present;
    time_ms last_time_board_ot_fault_not_present;
    time_ms last_time_pack_uv_fault_not_present;
    time_ms last_time_invalid_packet_present;
    time_ms prev_time_stamp;

    bool has_fault;
    bool charging_enabled;

    std::array<bool, num_cells> cell_balancing_statuses;
};

struct ACUControllerParameters {
    volt ov_thresh_v = 0;
    volt uv_thresh_v = 0;
    celsius charging_ot_thresh_c = 0;
    celsius running_ot_thresh_c = 0;
    volt min_pack_total_v = 0;
    size_t invalid_packet_count_thresh = 0;
    time_ms max_allowed_voltage_fault_dur = 0;
    time_ms max_allowed_temp_fault_dur = 0;
    time_ms max_allowed_invalid_packet_fault_dur = 0;
    volt v_diff_to_init_cb = 0;
};

template <size_t num_cells, size_t num_celltemps>
class ACUController
{
    using ACUData = etl::singleton<ACUData_s<num_cells, num_celltemps>>;
    using ACUStatus = ACUControllerData_s<num_cells>;
    
public:
   

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
                    size_t invalid_packet_count_thresh = acu_controller_default_params::MAX_INVALID_PACKET_FAULT_COUNT,
                    time_ms max_volt_fault_dur = acu_controller_default_params::MAX_VOLTAGE_FAULT_DUR,
                    time_ms max_temp_fault_dur = acu_controller_default_params::MAX_TEMP_FAULT_DUR,
                    time_ms max_invalid_packet_dur = acu_controller_default_params::MAX_INVALID_PACKET_FAULT_DUR,
                    volt v_diff_init_cb = acu_controller_default_params::VOLTAGE_DIFF_TO_INIT_CB) : 
        _parameters {
            ov_thresh_v,
            uv_thresh_v,
            charging_ot_thresh_c,
            running_ot_thresh_c,
            min_pack_total_voltage,
            invalid_packet_count_thresh,
            max_volt_fault_dur,
            max_temp_fault_dur,
            max_invalid_packet_dur,
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
    ACUStatus evaluate_accumulator(time_ms current_millis, const ACUData_s<num_cells, num_celltemps> &input_state);
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
     * @return boolean, true if there has been global invalid packets for > MAX DURATION
    */
    bool _check_invalid_packet_faults(time_ms current_millis);

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

    /**
     * @brief ACU Controller Parameters holder
    */
    const ACUControllerParameters _parameters = {};
};

template<size_t num_cells, size_t num_celltemps>
using ACUControllerInstance = etl::singleton<ACUController<num_cells, num_celltemps>>;

#include "ACUController.tpp"
#endif