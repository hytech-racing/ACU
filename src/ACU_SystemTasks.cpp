#include "ACU_SystemTasks.h"

/* Interface Function Dependencies */
#include "WatchdogInterface.h"

/* Delegate Function Definitions */
etl::delegate<bool()> received_CCU_message = etl::delegate<bool()>::create([]() -> bool {
    return false;
});

etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool {
    return ACUDataInstance::instance().acu_ok;
});

etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create<WatchdogInterface, &WatchdogInterface::read_imd_ok>(WatchdogInstance::instance());

etl::delegate<bool()> received_valid_shdn_out = etl::delegate<bool()>::create<WatchdogInterface, &WatchdogInterface::read_shdn_out>(WatchdogInstance::instance());

etl::delegate<void()> enable_cell_balancing = etl::delegate<void()>::create([]() -> void {
    ACUDataInstance::instance().charging_enabled = true;
});

etl::delegate<void()> disable_cell_balancing = etl::delegate<void()>::create([]() -> void {
    ACUDataInstance::instance().charging_enabled = false;
});

etl::delegate<void()> disable_watchdog = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_teensy_ok_low>(WatchdogInstance::instance());

etl::delegate<void()> reinitialize_watchdog = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_teensy_ok_high>(WatchdogInstance::instance());

etl::delegate<void()> disable_n_latch_en = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_n_latch_en_low>(WatchdogInstance::instance());

etl::delegate<void()> reset_latch = etl::delegate<void()>::create<WatchdogInterface, &WatchdogInterface::set_n_latch_en_high>(WatchdogInstance::instance());

bool initialize_all_systems() {
    // Initialize the ACU Controller
    ACUControllerInstance<NUM_CELLS>::create();
    ACUControllerInstance<NUM_CELLS>::instance().init(sys_time::hal_millis());
    /* State Machine Initialization */
    ACUStateMachineInstance::create(received_CCU_message,
                                    has_bms_fault,
                                    has_imd_fault,
                                    received_valid_shdn_out,
                                    enable_cell_balancing,
                                    disable_cell_balancing,
                                    disable_watchdog,
                                    reinitialize_watchdog,
                                    disable_n_latch_en,
                                    reset_latch);

    return true;
}

void evaluate_accumulator() {
    auto acu_status = ACUControllerInstance<NUM_CELLS>::instance().evaluate_accumulator(sys_time::hal_millis(), false, ACUDataInstance::instance()); // verified
    ACUDataInstance::instance().acu_ok = !acu_status.has_fault;
    ACUDataInstance::instance().cb = acu_status.cb;
}

void print_acu_status() {
    if (ACUDataInstance::instance().acu_ok) {
        Serial.print("ACU is OK\n");
    } else {
        Serial.print("ACU is NOT OK\n");
    }

    Serial.printf("Pack Voltage: %d\n", ACUDataInstance::instance().pack_voltage);
    Serial.printf("Minimum Cell Voltage: %d\t", ACUDataInstance::instance().min_cell_voltage);
    Serial.printf("Maximum Cell Voltage: %d\n", ACUDataInstance::instance().max_cell_voltage);
    Serial.printf("Maximum Board Temp: %d\t", ACUDataInstance::instance().max_board_temp);
    Serial.printf("Maximum Cell Temp: %d\n", ACUDataInstance::instance().max_cell_temp);

    Serial.println();
}

