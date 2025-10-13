#ifndef ACUCONTROLLER_H
#define ACUCONTROLLER_H

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/singleton.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include "ACU_Constants.h"

template <size_t num_cells>
struct ACUControllerData_s
{
    time_ms last_time_uv_fault_not_present;
    time_ms last_time_ov_fault_not_present;
    time_ms last_time_cell_ot_fault_not_present;
    time_ms last_time_board_ot_fault_not_present;
    time_ms last_time_pack_uv_fault_not_present;
    time_ms last_time_invalid_packet_present;
    time_ms prev_bms_time_stamp;
    time_ms prev_em_time_stamp;
    float SoC;
    bool has_fault;
    bool charging_enabled;
    bool balancing_enabled;
    std::array<bool, num_cells> cell_balancing_statuses;
};

struct ACUControllerParameters_s
{
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
    float pack_nominal_capacity = 0;
    float pack_max_voltage = 0;
    float pack_min_voltage = 0;
    celsius balance_temp_limit_c = 0;
    celsius balance_enable_temp_c = 0;
};

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
class ACUController
{
    using ACUData = etl::singleton<ACUData_s<num_cells, num_celltemps, num_boardtemps>>;
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
    ACUController(ACUControllerParameters_s params = {
                    .ov_thresh_v = ACUConstants::OV_THRESH,
                    .uv_thresh_v = ACUConstants::UV_THRESH,
                    .charging_ot_thresh_c = ACUConstants::CHARGING_OT_THRESH,
                    .running_ot_thresh_c = ACUConstants::RUNNING_OT_THRESH,
                    .min_pack_total_v = ACUConstants::MIN_PACK_TOTAL_VOLTAGE,
                    .invalid_packet_count_thresh = ACUConstants::MAX_INVALID_PACKET_FAULT_COUNT,
                    .max_allowed_voltage_fault_dur = ACUConstants::MAX_VOLTAGE_FAULT_DUR,
                    .max_allowed_temp_fault_dur = ACUConstants::MAX_TEMP_FAULT_DUR,
                    .max_allowed_invalid_packet_fault_dur = ACUConstants::MAX_INVALID_PACKET_FAULT_DUR,
                    .v_diff_to_init_cb = ACUConstants::VOLTAGE_DIFF_TO_INIT_CB,
                    .pack_nominal_capacity = ACUConstants::PACK_NOMINAL_CAPACITY_AH,
                    .pack_max_voltage = ACUConstants::PACK_MAX_VOLTAGE,
                    .pack_min_voltage = ACUConstants::PACK_MIN_VOLTAGE,
                    .balance_temp_limit_c = ACUConstants::BALANCE_TEMP_LIMIT_C,
                    .balance_enable_temp_c = ACUConstants::BALANCE_ENABLE_TEMP_THRESH_C
                }
            ) : _parameters(params) {};

    /**
     * @brief Initialize the status time stamps because we don't want accidental sudden faults
     */
    void init(time_ms system_start_time, volt pack_voltage);

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     * @param pack_current current flowing from the pack in amps (negative during discharge, positive during charge)
     */
    ACUStatus evaluate_accumulator(time_ms current_millis, const ACUData_s<num_cells, num_celltemps, num_boardtemps> &input_state, float pack_current);

    /**
     * @return state of charge - float from 0.0 to 1.0, representing a percentage from 0 to 100%
    */
    float get_state_of_charge(float em_current, uint32_t delta_time_ms);
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
private:
    /**
     * @brief Internal resistance per cell (computed from pack resistance divided by number of cells)
    */
    static constexpr float CELL_IR = ACUConstants::PACK_INTERNAL_RESISTANCE / static_cast<float>(num_cells);
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
    */
    ACUStatus _acu_state = {};

    /**
     * @brief ACU Controller Parameters holder
    */
    const ACUControllerParameters_s _parameters = {};
};

template<size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
using ACUControllerInstance = etl::singleton<ACUController<num_cells, num_celltemps, num_boardtemps>>;

#include "ACUController.tpp"
#endif