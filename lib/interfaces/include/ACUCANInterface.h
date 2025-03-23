#ifndef __ACU_CAN_INTERFACE_H__
#define __ACU_CAN_INTERFACE_H__

#include "etl/singleton.h"
#include "etl/delegate.h"

/* External Dependencies */
#include "SharedFirmwareTypes.h"

#include "hytech.h"
#include "CANInterface.h"


class ACUCANInterface {
public:
private:
};

using ACUCANInterfaceInstance = etl::singleton<ACUCANInterface>;

#endif