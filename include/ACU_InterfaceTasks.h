#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.h"
#include "shared_types.h"
#include "SharedFirmwareTypes.h"

/* Interface Library Includes */
#include "BMSDriverGroup.h"
#include "BMSFaultDataManager.h"
#include "WatchdogInterface.h"
#include "WatchdogMetrics.h"
#include "ACUEthernetInterface.h"
#include "ACUCANInterface.h"
#include "CCUInterface.h"
#include "ADCInterface.h"
#include "FaultLatchManager.h"
#include "SystemTimeInterface.h"
#include "ACUCANBuffers.h"
/* For Debugging */
#include "ACUStateMachine.h"

#include <ht_task.hpp>
#include <chrono>

using chip_type = LTC6811_Type_e;
using BMSDriverInstance_t = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>;
using BMSFaultDataManagerInstance_t = BMSFaultDataManagerInstance<ACUConstants::NUM_CHIPS>;
/**
 * Init Functions - to be called in setup
 */
void initialize_all_interfaces();

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
::HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_bms_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse write_cell_balancing_config(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_send_ACU_core_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

::HT_TASK::TaskResponse handle_send_ACU_all_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

::HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_all_voltages_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_all_temps_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse idle_sample_interfaces(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);


template <typename bms_data>
void print_bms_data(bms_data data);

#endif 