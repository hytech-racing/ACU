#ifndef ACU_GLOBALS
#define ACU_GLOBALS

#include "SharedFirmwareTypes.h"

#include "etl/optional.h"
#include "etl/singleton.h"
#include "etl/delegate.h"

#include "ACU_Constants.h"

using ACUDataInstance = etl::singleton<ACUData_s<NUM_CELLS>>;

struct ACU_Fault_Counter_s {
    size_t global_count;
    size_t consecutive_count;
};

using ACUFaultCountInstance = etl::singleton<ACU_Fault_Counter_s>;

#endif