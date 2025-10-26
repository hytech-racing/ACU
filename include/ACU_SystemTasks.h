#ifndef ACU_SYSTEMTASKS
#define ACU_SYSTEMTASKS

#include "ACU_Constants.h"
#include "shared_types.h"

/* Local System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Interface Function Dependencies */
#include "WatchdogInterface.h"
#include "CCUInterface.h"
#include "EMInterface.h"
#include "BMSDriverGroup.h"
#include "ADCInterface.h"
#include "SystemTimeInterface.h"
#include <ht_task.hpp>

using chip_type = LTC6811_Type_e;
using ACUControllerInstance_t = ACUControllerInstance<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>;
using BMSDriverInstance_t = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>;

bool initialize_all_systems();

/* Delegate Functions */
extern ::etl::delegate<bool()> received_CCU_message;
extern ::etl::delegate<bool()> has_bms_fault;
extern ::etl::delegate<bool()> has_imd_fault;
extern ::etl::delegate<bool()> received_valid_shdn_out;
extern ::etl::delegate<void()> enable_cell_balancing;
extern ::etl::delegate<void()> disable_cell_balancing;
extern ::etl::delegate<void()> disable_watchdog;
extern ::etl::delegate<void()> reinitialize_watchdog;
extern ::etl::delegate<void()> disable_n_latch_en;
extern ::etl::delegate<void()> reset_latch;

::HT_TASK::TaskResponse evaluate_accumulator(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

::HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

#endif 