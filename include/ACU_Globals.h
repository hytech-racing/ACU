#ifndef ACU_GLOBALS
#define ACU_GLOBALS

#include "SharedFirmwareTypes.h"

#include "etl/optional.h"
#include "etl/singleton.h"

#include "ACU_Constants.h"

// Should be reflected in SharedFirmwareTypes somewhat
template<size_t num_chips>
struct ACUData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages;
    celsius max_cell_temp; 
    celsius max_board_temp;

    bool acu_ok; // False when one of the three shutdown conditions is met (see AMSSystem header)
};

using ACUData = etl::singleton<ACUData_s<NUM_CHIPS>>;

#endif