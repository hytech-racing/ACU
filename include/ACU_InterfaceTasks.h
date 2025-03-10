#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"

/* Interface Library Includes */
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include "SystemTimeInterface.h"
#include <ht_task.hpp>

using chip_type = LTC6811_Type_e;

/**
 * Init Functions - to be called in setup
 */
void initialize_all_interfaces();

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

template <typename bms_data>
void print_bms_data(bms_data data);

template <typename bms_data>
void handle_bms_data(bms_data data);
/**
 * Needs to handle the logic and output for n_latch_en, and teensy_ok
 */
void handle_ACU_logic();

/* Miscellaneous debugging functions */
void set_bit(uint16_t & value, uint8_t index, bool bitValue);

#endif 