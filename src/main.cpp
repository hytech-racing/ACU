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

//etl::delegate<void(ACUCANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(ACUData_s &, const CAN_message_t &, unsigned long)>::create<ACUCANInterfaceImpl::acu_CAN_recv>();

HT_TASK::Task tick_state_machine_task(HT_TASK::DUMMY_FUNCTION, tick_state_machine, ACUConstants::TICK_SM_PRIORITY, ACUConstants::TICK_SM_PERIOD_US);
HT_TASK::Task kick_watchdog_task(HT_TASK::DUMMY_FUNCTION, run_kick_watchdog, ACUConstants::WATCHDOG_PRIORITY, ACUConstants::KICK_WATCHDOG_PERIOD_US); 
HT_TASK::Task sample_bms_data_task(HT_TASK::DUMMY_FUNCTION, sample_bms_data, ACUConstants::SAMPLE_BMS_PRIORITY, ACUConstants::SAMPLE_BMS_PERIOD_US);
HT_TASK::Task eval_accumulator_task(HT_TASK::DUMMY_FUNCTION, evaluate_accumulator, ACUConstants::EVAL_ACC_PRIORITY, ACUConstants::EVAL_ACC_PERIOD_US);
HT_TASK::Task write_cell_balancing_config_task(HT_TASK::DUMMY_FUNCTION, write_cell_balancing_config, ACUConstants::WRITE_CELL_BALANCE_PRIORITY, ACUConstants::WRITE_CELL_BALANCE_PERIOD_US);
HT_TASK::Task send_all_data_ethernet_task(HT_TASK::DUMMY_FUNCTION, handle_send_ACU_all_ethernet_data, ACUConstants::ALL_DATA_ETHERNET_PRIORITY, ACUConstants::ALL_DATA_ETHERNET_PERIOD_US);
HT_TASK::Task send_core_data_ethernet_task(HT_TASK::DUMMY_FUNCTION, handle_send_ACU_core_ethernet_data, ACUConstants::CORE_DATA_ETHERNET_PRIORITY, ACUConstants::CORE_DATA_ETHERNET_PERIOD_US);
HT_TASK::Task send_CAN_task(HT_TASK::DUMMY_FUNCTION, handle_send_all_CAN_data, ACUConstants::SEND_CAN_PRIORITY, ACUConstants::SEND_CAN_PERIOD_US);
HT_TASK::Task enqueue_CCU_core_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_CCU_core_CAN_data, ACUConstants::CCU_SEND_PRIORITY, ACUConstants::CCU_SEND_PERIOD_US);
HT_TASK::Task enqueue_CCU_subA_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_CCU_sub_A_CAN_data, ACUConstants::CCU_SEND_A_PRIORITY, ACUConstants::CCU_SEND_A_PERIOD_US);
HT_TASK::Task enqueue_CCU_subB_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_CCU_sub_B_CAN_data, ACUConstants::CCU_SEND_B_PRIORITY, ACUConstants::CCU_SEND_B_PERIOD_US);

HT_TASK::Task sample_CAN_task(HT_TASK::DUMMY_FUNCTION, sample_CAN_data, ACUConstants::RECV_CAN_PRIORITY, ACUConstants::RECV_CAN_PERIOD_US);

HT_TASK::Task debug_prints_task(HT_TASK::DUMMY_FUNCTION, debug_print, ACUConstants::DEBUG_PRINT_PRIORITY, ACUConstants::DEBUG_PRINT_PERIOD_US);

FlexCAN_Type<CAN3> CCU_CAN;

void setup()
{
    /* Interface and System initialization */
    initialize_all_interfaces();
    initialize_all_systems();

    scheduler.setTimingFunction(micros);
    scheduler.schedule(tick_state_machine_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(sample_bms_data_task);
    scheduler.schedule(eval_accumulator_task);
    //scheduler.schedule(write_cell_balancing_config_task);

    //scheduler.schedule(send_all_data_ethernet_task);
    //scheduler.schedule(send_core_data_ethernet_task);

    scheduler.schedule(send_CAN_task);
    scheduler.schedule(enqueue_CCU_core_CAN_task);
    scheduler.schedule(enqueue_CCU_subA_CAN_task);
    scheduler.schedule(enqueue_CCU_subB_CAN_task);

    scheduler.schedule(sample_CAN_task);

    scheduler.schedule(debug_prints_task);

    handle_CAN_setup(CCU_CAN, ACUConstants::CAN_baudrate, &ACUCANInterfaceImpl::on_ccu_can_receive);
}

void loop()
{  
    scheduler.run();
}