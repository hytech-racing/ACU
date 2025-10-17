#include "ACU_SystemTasks.h"

/* Interface Function Dependencies */
#include "WatchdogInterface.h"
#include "CCUInterface.h"
#include "EMInterface.h"

const constexpr uint32_t bms_not_ok_hold_time_ms = 1000;

bool initialize_all_systems()
{
    // Initialize the ACU Controller
    ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::create(ACUControllerThresholds_s{ACUSystems::CELL_OVERVOLTAGE_THRESH,
                                                                                                                                                  ACUSystems::CELL_UNDERVOLTAGE_THRESH,
                                                                                                                                                  ACUSystems::CHARGING_OT_THRESH,
                                                                                                                                                  ACUSystems::RUNNING_OT_THRESH,
                                                                                                                                                  ACUSystems::MIN_PACK_TOTAL_VOLTAGE,
                                                                                                                                                  ACUSystems::VOLTAGE_DIFF_TO_INIT_CB,
                                                                                                                                                  ACUSystems::BALANCE_TEMP_LIMIT_C,
                                                                                                                                                  ACUSystems::BALANCE_ENABLE_TEMP_THRESH_C});
    ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::instance().init(sys_time::hal_millis(), ACUDataInstance::instance().pack_voltage);
    /* State Machine Initialization */

    /* Delegate Function Definitions */
    etl::delegate<bool()> charge_state_request = etl::delegate<bool()>::create([]() -> bool
                                                                               { return CCUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).charging_requested; });

    etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ACUDataInstance::instance().bms_ok; });

    etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !WatchdogInstance::instance().read_imd_ok(sys_time::hal_millis()); });

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
                                    disable_n_latch_en,
                                    sys_time::hal_millis());

    return true;
}

HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    // Fetch EM current data first for IR compensation
    EMData_s em_data = EMInterfaceInstance::instance().get_latest_data(sys_time::hal_millis());

    // Evaluate accumulator with IR compensation
    auto acu_status = ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::instance().evaluate_accumulator(sys_time::hal_millis(), ACUDataInstance::instance(), em_data.em_current);

    if (acu_status.has_fault)
    {
        ACUDataInstance::instance().bms_ok = !acu_status.has_fault;
        ACUDataInstance::instance().last_bms_not_ok_eval = sys_time::hal_millis();
    }
    else if (!ACUDataInstance::instance().bms_ok && (sys_time::hal_millis() - ACUDataInstance::instance().last_bms_not_ok_eval > bms_not_ok_hold_time_ms))
    {
        ACUDataInstance::instance().bms_ok = true;
    }
    ACUDataInstance::instance().cell_balancing_statuses = acu_status.cell_balancing_statuses;
    ACUAllDataInstance::instance().SoC = ACUDataInstance::instance().SoC = ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>::instance().get_state_of_charge(em_data.em_current, em_data.time_since_prev_msg_ms);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}
