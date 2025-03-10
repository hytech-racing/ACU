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
extern etl::delegate<bool()> mock_hv_over_threshold;


#endif 