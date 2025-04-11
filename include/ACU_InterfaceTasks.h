#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"
#include "shared_types.h"

/* Interface Library Includes */
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include "SystemTimeInterface.h"
#include "ACUEthernetInterface.h"
#include "ACUCANInterface.h"

/* For Debugging */
#include "ACUStateMachine.h"

#include <ht_task.hpp>

/**
 * Init Functions - to be called in setup
 */
void initialize_all_interfaces();

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool sample_bms_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool write_cell_balancing_config(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool handle_send_ACU_core_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

bool handle_send_ACU_all_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

bool handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool enqueue_ACU_sub_A_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool enqueue_ACU_sub_B_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool sample_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

bool debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

template <typename bms_data>
void print_bms_data(bms_data data);

#endif 