#include "ACU_SystemTasks.h"

bool initialize_all_systems()
{
    // Initialize the ACU Controller
    ACUControllerInstance::create(ACUControllerThresholds_s{  ACUSystems::MIN_DISCHARGE_VOLTAGE_THRESH,
                                                                ACUSystems::CELL_OVERVOLTAGE_THRESH,
                                                                ACUSystems::CELL_UNDERVOLTAGE_THRESH,
                                                                ACUSystems::CHARGING_OT_THRESH,
                                                                ACUSystems::RUNNING_OT_THRESH,
                                                                ACUSystems::MIN_PACK_TOTAL_VOLTAGE,
                                                                ACUSystems::VOLTAGE_DIFF_TO_INIT_CB,
                                                                ACUSystems::BALANCE_TEMP_LIMIT_C,
                                                                ACUSystems::BALANCE_ENABLE_TEMP_THRESH_C});
    ACUControllerInstance::instance().init(sys_time::hal_millis(), BMSDriverInstance_t::instance().get_bms_data().total_voltage);
    /* State Machine Initialization */

    /* Delegate Function Definitions */
    etl::delegate<bool()> charge_state_request = etl::delegate<bool()>::create([]() -> bool
                                                                            { return CCUInterfaceInstance::instance().is_charging_with_balancing_requested(sys_time::hal_millis()); });

    etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ACUControllerInstance::instance().get_status().bms_ok; });

    etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ADCInterfaceInstance::instance().read_imd_ok(sys_time::hal_millis()); });

    etl::delegate<bool()> received_valid_shdn_out = etl::delegate<bool()>::create<ADCInterface, &ADCInterface::read_shdn_out>(ADCInterfaceInstance::instance());

    etl::delegate<void()> enable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUControllerInstance::instance().enableCharging(); });

    etl::delegate<void()> disable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUControllerInstance::instance().disableCharging(); });
    etl::delegate<void()> disable_watchdog = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_teensy_ok_low>(WatchdogInstance::instance());

    etl::delegate<void()> reinitialize_watchdog = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_teensy_ok_high>(WatchdogInstance::instance());

    etl::delegate<void()> disable_n_latch_en = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_n_latch_en_low>(WatchdogInstance::instance());

    etl::delegate<void()> reset_latch = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_n_latch_en_high>(WatchdogInstance::instance());

    ACUStateMachineInstance::create(charge_state_request,
                                    has_bms_fault,
                                    has_imd_fault,
                                    received_valid_shdn_out,
                                    enable_cell_balancing,
                                    disable_cell_balancing,
                                    disable_watchdog,
                                    reinitialize_watchdog,
                                    reset_latch,
                                    disable_n_latch_en,
                                    sys_time::hal_millis());

    return true;
}

HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUControllerInstance::instance().evaluate_accumulator(
        sys_time::hal_millis(), 
        BMSDriverInstance_t::instance().get_bms_core_data(), 
        BMSFaultDataManagerInstance_t::instance().get_fault_data().max_consecutive_invalid_packet_count,
        EMInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).em_current,
        ACUConstants::NUM_CELLS
    );
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}
