#ifndef ACU_GLOBALS
#define ACU_GLOBALS

#include "SharedFirmwareTypes.h"

#include "etl/optional.h"
#include "etl/singleton.h"
#include "etl/delegate.h"

#include "ACU_Constants.h"

using ACUDataInstance = etl::singleton<ACUData_s<NUM_CELLS>>;

#endif