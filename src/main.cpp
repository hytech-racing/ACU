/* ACU Dependent */
#include "ACU_Constants.h"
#include "ACU_Globals.h"
#include "ACU_InterfaceTasks.h"
#include "ACU_SystemTasks.h"

/* Interface Includes */
#include <Arduino.h>
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include "SystemTimeInterface.h"
// #include "ACUEthernetInterface.h"

/* System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

void setup()
{
    /* Interface and System initialization */
    initialize_all_interfaces();
    initialize_all_systems();
}

void loop()
{      
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis()); // verified 

    if (sys_time::hal_millis() % 200 == 0) { // 5Hz
        BMSData data = BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().read_data(); // verified

        auto acu_status = ACUControllerInstance<NUM_CELLS>::instance().evaluate_accumulator(sys_time::hal_millis(), false, ACUDataInstance::instance()); // verified
        ACUDataInstance::instance().acu_ok = acu_status.has_fault;
        Serial.print(acu_status.has_fault);
        BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().write_configuration(dcto_write, acu_status.cb);
    }
    
    if (sys_time::hal_millis() % 500 == 0) { // 10 Hz
        auto state = ACUStateMachineInstance::instance().get_state();
        Serial.printf("state: %d\n", static_cast<int>(state));
    }
    
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    // scheduler.run(); 
}