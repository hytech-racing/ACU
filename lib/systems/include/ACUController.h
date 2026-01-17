#ifndef ACUCONTROLLER_H
#define ACUCONTROLLER_H

#include <array>
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
    bool bms_ok;
    uint32_t last_bms_not_ok_eval;
    bool charging_enabled;
    bool balancing_enabled;
};

struct ACUControllerThresholds_s
{
    volt min_discharge_voltage_thresh = 0;
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
    float pack_internal_resistance = 0;
};

struct ACUControllerParameters_s
{
    ACUControllerThresholds_s thresholds;
    size_t invalid_packet_count_thresh = 0;
    ACUControllerFaultDurations_s fault_durations;
    ACUControllerPackSpecs_s pack_specs;
};
class ACUController
{
    // using ACUData = etl::singleton<BMSCoreData_s<num_cells, num_celltemps, num_boardtemps>>;

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
                    .pack_internal_resistance = acu_controller_default_parameters::PACK_INTERNAL_RESISTANCE
                }) : _acu_parameters{thresholds, invalid_packet_count_thresh, fault_durations, pack_specs} {};

    /**
     * @brief Initialize the status time stamps because we don't want accidental sudden faults
     */
    void init(time_ms system_start_time, volt pack_voltage);

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     * @param pack_current current flowing from the pack in amps (negative during discharge, positive during charge)
     */
    ACUControllerData_s evaluate_accumulator(time_ms current_millis, const BMSCoreData_s &bms_core_data, size_t max_consecutive_invalid_packet_count, float em_current, size_t num_of_voltage_cells);

        /**
     * Calculate Cell Balancing values
     * @pre cell charging is enabled
     * @post output will have the new values
     */
    void calculate_cell_balance_statuses(bool* output, const volt* voltages, size_t num_of_voltage_cells, volt min_voltage);

    /**
     * @return state of charge - float from 0.0 to 1.0, representing a percentage from 0 to 100%
     */
    float get_state_of_charge(float em_current, uint32_t delta_time_ms);

    ACUControllerData_s get_status() const { return _acu_state; };

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
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we ican feed it directly into the message interface as a struct
     */
    ACUControllerData_s _acu_state = {};

    static constexpr uint32_t _bms_not_ok_hold_time_ms = 1000;

    static constexpr uint32_t _ms_to_hours = 3600000;

    /**
     * @brief ACU Controller Parameters holder
     */
    const ACUControllerParameters_s _acu_parameters = {};
};

using ACUControllerInstance = etl::singleton<ACUController>;

#endif