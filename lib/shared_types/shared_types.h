#ifndef SHAREDTYPES_H
#define SHAREDTYPES_H

#include "SharedFirmwareTypes.h"

template<size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
struct BMSCoreData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    std::array<volt, num_cells> voltages;
    celsius max_cell_temp; 
    celsius min_cell_temp; 
    celsius max_board_temp;
    size_t max_consecutive_invalid_packet_count;
    bool charging_enabled;
};

#endif
