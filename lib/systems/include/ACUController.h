#ifndef ACUCONTROLLER_H
#define ACUCONTROLLER_H

#include <array>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/singleton.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

namespace acu_controller_default_parameters
{
    constexpr const size_t MAX_INVALID_PACKET_FAULT_COUNT = 1000000; // Same as voltage fault count
    constexpr const time_ms MAX_VOLTAGE_FAULT_DUR = 1000;            // At 15 Hz, we'll know if there is an error within 3 seconds of startup
    constexpr const time_ms MAX_TEMP_FAULT_DUR = 1000;
    constexpr const time_ms MAX_INVALID_PACKET_FAULT_DUR = 500; // In cases in EMI, we will need more leniency with invalid packet faults

    constexpr const float PACK_NOMINAL_CAPACITY_AH = 13.5;  // nominal pack capacity in amp * hours
    constexpr const float PACK_MAX_VOLTAGE = 529.2;         // from data sheet https://wiki.hytechracing.org/books/ht09-design/page/molicel-pack-investigation
    constexpr const float PACK_MIN_VOLTAGE = 378.0;         // from data sheet^ but just assume 126 * 3.0V
    constexpr const float PACK_INTERNAL_RESISTANCE = 0.246; // Ohms (measured)
}

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
    time_ms first_zero_current_time_stamp;
    float SoC;
    bool has_fault;
    bool bms_ok;
    uint32_t last_bms_not_ok_eval;
    bool charging_enabled;
    bool balancing_enabled;
    std::array<bool, num_cells> cell_balancing_statuses;
};

struct ACUControllerThresholds_s
{
    volt cell_overvoltage_thresh_v = 0;
    volt cell_undervoltage_thresh_v = 0;
    celsius charging_ot_thresh_c = 0;
    celsius running_ot_thresh_c = 0;
    volt min_pack_total_v = 0;
    volt v_diff_to_init_cb = 0;
    celsius balance_temp_limit_c = 0;
    celsius balance_enable_temp_c = 0;
};

struct ACUControllerFaultDurations_s
{
    time_ms max_allowed_voltage_fault_dur = 0;
    time_ms max_allowed_temp_fault_dur = 0;
    time_ms max_allowed_invalid_packet_fault_dur = 0;
};

struct ACUControllerPackSpecs_s
{
    float pack_nominal_capacity = 0;
    float pack_max_voltage = 0;
    float pack_min_voltage = 0;
};

struct ACUControllerParameters_s
{
    ACUControllerThresholds_s thresholds;
    size_t invalid_packet_count_thresh = 0;
    ACUControllerFaultDurations_s fault_durations;
    ACUControllerPackSpecs_s pack_specs;
};
template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
class ACUController
{
    using ACUData = etl::singleton<BMSCoreData_s<num_cells, num_celltemps, num_boardtemps>>;
    using ACUStatus = ACUControllerData_s<num_cells>;

public:
    /**
     * ACU Controller Constructor
     * @param cell_overvoltage_thresh_v cell overvoltage threshold value | units in volts
     * @param cell_undervoltage_thresh_v cell undervoltage threshold value | units in volts
     * @param charging_ot_thresh_c overtemp threshold value | units in celsius
     * @param funning_ot_thresh_c overtemp threshold value | units in celsius
     * @param min_pack_total_voltage minimum pack total voltage | units in volts
     * @param max_volt_fault_dur max number of voltage faults allowed
     * @param max_temp_fault_dur max number of temp faults allowed
     */
    ACUController(ACUControllerThresholds_s thresholds,
                  size_t invalid_packet_count_thresh = acu_controller_default_parameters::MAX_INVALID_PACKET_FAULT_COUNT,
                  ACUControllerFaultDurations_s fault_durations = {
                      .max_allowed_voltage_fault_dur = acu_controller_default_parameters::MAX_VOLTAGE_FAULT_DUR,
                      .max_allowed_temp_fault_dur = acu_controller_default_parameters::MAX_TEMP_FAULT_DUR,
                      .max_allowed_invalid_packet_fault_dur = acu_controller_default_parameters::MAX_INVALID_PACKET_FAULT_DUR},
                  ACUControllerPackSpecs_s pack_specs = {
                    .pack_nominal_capacity = acu_controller_default_parameters::PACK_NOMINAL_CAPACITY_AH,
                    .pack_max_voltage = acu_controller_default_parameters::PACK_MAX_VOLTAGE,
                    .pack_min_voltage = acu_controller_default_parameters::PACK_MIN_VOLTAGE,
                    }
                ) : _acu_parameters{thresholds, invalid_packet_count_thresh, fault_durations, pack_specs} {};

    /**
     * @brief Initialize the status time stamps because we don't want accidental sudden faults
     */
    void init(time_ms system_start_time, volt pack_voltage);

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     * @param pack_current current flowing from the pack in amps (negative during discharge, positive during charge)
     */
    ACUStatus evaluate_accumulator(time_ms current_millis, const BMSCoreData_s<num_cells, num_celltemps, num_boardtemps> &input_state, float em_current);

    /**
     * @return state of charge - float from 0.0 to 1.0, representing a percentage from 0 to 100%
     */
    float get_state_of_charge(float em_current, uint32_t delta_time_ms, volt avg_cell_voltage, time_ms current_millis);

    ACUStatus get_status() const { return _acu_state; };

    void enableCharging()
    {
        _acu_state.charging_enabled = true;
    }
    void disableCharging()
    {
        _acu_state.charging_enabled = false;
    }
private:
    /**
     * Calculate Cell Balancing values
     * @pre cell charging is enabled
     * @post _acu_state.cell_balance_statuses will have the new values
     */
    std::array<bool, num_cells> _calculate_cell_balance_statuses(std::array<volt, num_cells> voltages, volt min_voltage);

        /**
     * @brief Closest index that will represent the SoC of the average voltage on the cells
     */
    uint16_t _get_soc_from_voltage(volt avg_cell_voltage);

    /**
     * @pre data has been gathered
     * @return boolean, true if there exists any fault
     */
    bool _check_faults(time_ms current_millis);

    /**
     * @brief Update the BMS status (bms_ok) based on the time since the last fault not present
     */
    bool _check_bms_ok(time_ms current_millis);
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
    static constexpr float CELL_INTERNAL_RESISTANCE = acu_controller_default_parameters::PACK_INTERNAL_RESISTANCE / static_cast<float>(num_cells);
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
     */
    ACUStatus _acu_state = {};

    static constexpr uint32_t _bms_not_ok_hold_time_ms = 1000;

    /**
     * @brief ACU Controller Parameters holder
     */
    const ACUControllerParameters_s _acu_parameters = {};

    /**
     * @brief OCV-SOC Lookup Table from MCU on HT-08: voltage values corresponding to SOC from 0% to 100% 
     * 
     */
    static constexpr float VOLTAGE_LOOKUP_TABLE[101] = {3.972, 3.945, 3.918, 3.891, 3.885, 3.874, 3.864, 3.858, 3.847, 3.836, 3.82, 3.815, 3.815, 3.798, 3.788,
        3.782, 3.771, 3.755, 3.744, 3.744, 3.733, 3.728, 3.723, 3.712, 3.701, 3.695, 3.69, 3.679, 3.679, 3.668, 3.663, 3.657, 3.647,
        3.647, 3.636, 3.625, 3.625, 3.625, 3.614, 3.609, 3.603, 3.603, 3.592, 3.592, 3.592, 3.581, 3.581, 3.571, 3.571, 3.571, 3.56,
        3.56, 3.56, 3.549, 3.549, 3.549, 3.549, 3.538, 3.538, 3.551, 3.546, 3.535, 3.535, 3.535, 3.53, 3.524, 3.524, 3.524, 3.513,
        3.513, 3.513, 3.503, 3.503, 3.492, 3.492, 3.492, 3.487, 3.481, 3.481, 3.476, 3.471, 3.46, 3.46, 3.449, 3.444, 3.428, 3.428,
        3.417, 3.401, 3.39, 3.379, 3.363, 3.331, 3.299, 3.267, 3.213, 3.149, 3.041, 3, 3, 0};

    /**
     * @brief Minimum current and voltage thresholds for the car to be considered stabilized
     * 
     */
    static constexpr float STABILIZED_CURRENT_THRESH = 0.05; // Absolute value threshold
    static constexpr uint32_t MIN_STABILIZED_CURRENT_DURATION_MS = 1800000;  // 30 minutes in milliseconds
};

template <size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
using ACUControllerInstance = etl::singleton<ACUController<num_cells, num_celltemps, num_boardtemps>>;

#include "ACUController.tpp"
#endif