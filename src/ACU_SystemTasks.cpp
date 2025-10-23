#include "ACU_SystemTasks.h"

/* Interface Function Dependencies */
#include "WatchdogInterface.h"
#include "CCUInterface.h"
#include "EMInterface.h"
#include "BMSDriverGroup.h"

using chip_type = LTC6811_Type_e;
using ACUControllerInstance_t = ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>;
using BMSDriverInstance_t = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>;

bool initialize_all_systems()
{
    // Initialize the ACU Controller
    ACUControllerInstance_t::create();
    ACUControllerInstance_t::instance().init(sys_time::hal_millis(), BMSDriverInstance_t::instance().get_data().total_voltage);
    /* State Machine Initialization */

    /* Delegate Function Definitions */
    etl::delegate<bool()> charge_state_request = etl::delegate<bool()>::create([]() -> bool
                                                                            { return CCUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).charging_requested; });

    etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !ACUControllerInstance_t::instance().get_status().bms_ok; });

    etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool
                                                                        { return !WatchdogInstance::instance().read_imd_ok(sys_time::hal_millis()); });

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
                                    sys_time::hal_millis());

    return true;
}

HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUControllerInstance_t::instance().evaluate_accumulator(
        sys_time::hal_millis(), 
        BMSDriverInstance_t::instance().get_acu_data(), 
        EMInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).em_current
    );
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo) {
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}

