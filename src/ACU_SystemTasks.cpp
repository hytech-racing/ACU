#include "ACU_SystemTasks.h"

bool initialize_all_systems()
{
    // Initialize the ACU Controller
    ACUControllerInstance_t::create();
    ACUControllerInstance_t::instance().init(millis(), BMSDriverInstance_t::instance().get_bms_data().total_voltage);
    /* State Machine Initialization */

    /* Delegate Function Definitions */
    etl::delegate<bool()> charge_state_request = etl::delegate<bool()>::create([]() -> bool
                                                                            { return CCUInterfaceInstance::instance().get_latest_data(millis()).charging_requested; });

    etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ACUControllerInstance_t::instance().get_status().bms_ok; });

    etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !WatchdogInstance::instance().read_imd_ok(millis()); });

    etl::delegate<bool()> received_valid_shdn_out = etl::delegate<bool()>::create<WatchdogInterface, &WatchdogInterface::read_shdn_out>(WatchdogInstance::instance());

    etl::delegate<void()> enable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUControllerInstance_t::instance().enableCharging(); });

    etl::delegate<void()> disable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUControllerInstance_t::instance().disableCharging(); });
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
                                    millis());

    return true;
}

HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUControllerInstance_t::instance().evaluate_accumulator(
        millis(), 
        BMSDriverInstance_t::instance().get_bms_core_data(), 
        EMInterfaceInstance::instance().get_latest_data(millis()).em_current
    );
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo) {
    ACUStateMachineInstance::instance().tick_state_machine(millis());

    return HT_TASK::TaskResponse::YIELD;
}

