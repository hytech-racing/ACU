#ifndef ACU_SYSTEMTASKS
#define ACU_SYSTEMTASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"
#include "shared_types.h"

/* Local System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"
#include "SystemTimeInterface.h"

#include <ht_task.hpp>

bool initialize_all_systems();

/* Delegate Functions */
extern etl::delegate<bool()> received_CCU_message;
extern etl::delegate<bool()> has_bms_fault;
extern etl::delegate<bool()> has_imd_fault;
extern etl::delegate<bool()> received_valid_shdn_out;

extern etl::delegate<void()> enable_cell_balancing;
extern etl::delegate<void()> disable_cell_balancing;
extern etl::delegate<void()> disable_watchdog;
extern etl::delegate<void()> reinitialize_watchdog;
extern etl::delegate<void()> disable_n_latch_en;
extern etl::delegate<void()> reset_latch;

HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

#endif 