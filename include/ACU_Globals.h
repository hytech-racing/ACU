#ifndef ACU_GLOBALS
#define ACU_GLOBALS

#include "SharedFirmwareTypes.h"

#include "etl/optional.h"
#include "etl/singleton.h"

#include "ACU_Constants.h"

// Should be reflected in SharedFirmwareTypes somewhat
template<size_t num_cells>
struct ACUData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    std::array<volt, num_cells> voltages;
    celsius max_cell_temp; 
    celsius max_board_temp;

    bool charging_enabled;
    bool acu_ok; // False when one of the three shutdown conditions is met (see AMSSystem header)
};

using ACUData = etl::singleton<ACUData_s<NUM_CELLS>>;

#endif