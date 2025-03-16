#ifndef ACU_SYSTEMTASKS
#define ACU_SYSTEMTASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"

/* Local System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"
#include "SystemTimeInterface.h"

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

void evaluate_accumulator();

void print_acu_status();

#endif 