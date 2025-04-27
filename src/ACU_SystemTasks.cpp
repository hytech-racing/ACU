#include "ACU_SystemTasks.h"

/* Interface Function Dependencies */
#include "WatchdogInterface.h"
#include "CCUInterface.h"

bool initialize_all_systems()
{
    // Initialize the ACU Controller
    ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::create();
    ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::instance().init(sys_time::hal_millis());
    /* State Machine Initialization */

    /* Delegate Function Definitions */
    etl::delegate<bool()> charge_state_request = etl::delegate<bool()>::create([]() -> bool
                                                                            { return CCUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).charging_requested; });

    etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ACUDataInstance::instance().acu_ok; });

    etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !WatchdogInstance::instance().read_imd_ok(); });

    etl::delegate<bool()> received_valid_shdn_out = etl::delegate<bool()>::create<WatchdogInterface, &WatchdogInterface::read_shdn_out>(WatchdogInstance::instance());

    etl::delegate<void()> enable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUDataInstance::instance().charging_enabled = true; });

    etl::delegate<void()> disable_cell_balancing = etl::delegate<void()>::create([]() -> void
                                                                                { ACUDataInstance::instance().charging_enabled = false; });
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
                                    disable_n_latch_en);

    return true;
}

bool evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    auto acu_status = ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::instance().evaluate_accumulator(sys_time::hal_millis(), ACUDataInstance::instance()); // verified
    ACUDataInstance::instance().acu_ok = !acu_status.has_fault;
    ACUDataInstance::instance().cell_balancing_statuses = acu_status.cell_balancing_statuses;

    return true;
}

bool tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo) {
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    return true;
}

