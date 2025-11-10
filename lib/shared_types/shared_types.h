#ifndef SHAREDTYPES_H
#define SHAREDTYPES_H

#include "SharedFirmwareTypes.h"

struct BMSCoreData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    celsius max_cell_temp; 
    celsius min_cell_temp; 
    celsius max_board_temp;
};

#endif
