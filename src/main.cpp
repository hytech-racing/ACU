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

/* System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

using namespace qindesign::network;
EthernetUDP acu_core_data_send_socket; 
EthernetUDP acu_all_data_send_socket; 
EthernetUDP vcr_data_recv_socket;
EthernetUDP db_data_recv_socket; 

void setup()
{
    /* Interface and System initialization */
    initialize_all_interfaces();
    initialize_all_systems();

    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().acu_ip,  EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    acu_core_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    acu_all_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);
    vcr_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    db_data_recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);
}

void loop()
{      
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis()); // verified 

    /* BMS Data acquisition | Cell Balancing (CB) Calculation | BMS CB Writing IF Charging */
    if (sys_time::hal_millis() % 200 == 0) { // 5Hz
        get_bms_data();

        evaluate_accumulator();
        
        write_cell_balancing_config();

        /* Prints */
        print_watchdog_data();
        print_acu_status();
    }
    
    /* State Machine Tick */
    ACUStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

    if (sys_time::hal_millis() % 100 == 0) { // 10 Hz
        // UDP Message Send
        
    }

    if (sys_time::hal_millis() % 200 == 0) { // 5 Hz
        // TCP Message send

    }

    // scheduler.run(); 
}