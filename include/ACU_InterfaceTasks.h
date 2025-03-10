#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"

/* Interface Library Includes */
#include "BMSDriverGroup.h"
#include <ht_task.hpp>

using chip_type = LTC6811_Type_e;

/**
 * Init Functions - to be called in setup
 */
 
void initialize_interfaces();
void initialize_systems();

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
bool init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


/* Miscellaneous debugging functions */
void set_bit(uint16_t & value, uint8_t index, bool bitValue);

#endif 