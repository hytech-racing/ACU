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

    delay(500);
    Serial.println("Setup Complete");
    delay(500);
}

void loop()
{      
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis()); // verified 

    if (sys_time::hal_millis() % 200 == 0) { // 5Hz
        auto data = BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().read_data(); // verified
        print_bms_data(data);

        ACUControllerInstance<NUM_CELLS>::instance().evaluate_accumulator(sys_time::hal_millis(), false, ACUData); 
    }
    
    
    
    // scheduler.run(); 
}