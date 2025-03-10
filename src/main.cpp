/* ACU Dependent */
#include "ACU_Constants.h"
#include "ACU_Globals.h"
#include "ACU_InterfaceTasks.h"
#include "ACU_SystemTasks.h"

/* Interface Includes */
#include <Arduino.h>
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include "ACUEthernetInterface.h"

/* System Includes */
#include "ACUController.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

// Instantiate BMS Driver Group
BMSDriverGroup<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>(CS, CS_PER_CHIP, ADDR);

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

void setup()
{
    BMSGroup.init();
    
}

void loop()
{
    scheduler.run(); 
}