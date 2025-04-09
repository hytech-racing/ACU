#ifndef ACU_GLOBALS
#define ACU_GLOBALS

#include "SharedFirmwareTypes.h"

#include "etl/optional.h"
#include "etl/singleton.h"
#include "etl/delegate.h"

#include "ACU_Constants.h"

#include "shared_types.h"
#include "FlexCAN_T4.h"

using ACUFaultDataInstance = etl::singleton<ACUFaultData_s<ACUConstants::NUM_CHIPS>>;
using ACUDataInstance = etl::singleton<ACUData_s<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS>>;
using ACUAllDataInstance = etl::singleton<ACUAllDataType_s>;

using ACUAllDataType_s = ACUAllData_s<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_CHIPS>;

#endif