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
#include "ACUEthernetInterface.h"
#include "ACUCANInterface.h"

/* System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* externed CAN instances */
//FlexCAN_Type<CAN2> ACUCANInterfaceImpl::CCU_CAN;

//etl::delegate<void(ACUCANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(ACUData_s &, const CAN_message_t &, unsigned long)>::create<ACUCANInterfaceImpl::acu_CAN_recv>();

HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, run_kick_watchdog, ACUConstants::WATCHDOG_PRIORITY, ACUConstants::KICK_WATCHDOG_PERIOD_US); 
HT_TASK::Task sample_bms_data_task(HT_TASK::DUMMY_FUNCTION, sample_bms_data, ACUConstants::SAMPLE_BMS_PRIORITY, ACUConstants::SAMPLE_BMS_PERIOD_US);
HT_TASK::Task eval_accumulator_task(HT_TASK::DUMMY_FUNCTION, evaluate_accumulator, ACUConstants::EVAL_ACC_PRIORITY, ACUConstants::EVAL_ACC_PERIOD_US);
HT_TASK::Task write_cell_balancing_config_task(HT_TASK::DUMMY_FUNCTION, write_cell_balancing_config, ACUConstants::WRITE_CELL_BALANCE_PRIORITY, ACUConstants::WRITE_CELL_BALANCE_PERIOD_US);
HT_TASK::Task debug_prints_task(HT_TASK::DUMMY_FUNCTION, write_cell_balancing_config, ACUConstants::WRITE_CELL_BALANCE_PRIORITY, ACUConstants::WRITE_CELL_BALANCE_PERIOD_US);


void setup()
{
    /* Interface and System initialization */
    initialize_all_interfaces();
    initialize_all_systems();

    scheduler.setTimingFunction(micros);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(sample_bms_data_task);
    scheduler.schedule(eval_accumulator_task);
    scheduler.schedule(write_cell_balancing_config_task);
    //scheduler.schedule(debug_prints_task);
}

void loop()
{  
    /* BMS Data acquisition | Cell Balancing (CB) Calculation | BMS CB Writing IF Charging */
    if (sys_time::hal_millis() % 200 == 0) { // 5Hz
        //sample_bms_data();
        //evaluate_accumulator();
        //write_cell_balancing_config();

        /* Debug Prints */
        print_watchdog_data();
        print_acu_status();
    }
    
    /* State Machine Tick */
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    // if (sys_time::hal_millis() % 100 == 0) { // 10 Hz
    //     // UDP Message Send
    //     handle_send_ACU_core_ethernet_data();
    // }

    // if (sys_time::hal_millis() % 200 == 50) { // 5 Hz
    //     // TCP Message send
    //     handle_send_ACU_all_ethernet_data();
    // }

    scheduler.run(); 
}