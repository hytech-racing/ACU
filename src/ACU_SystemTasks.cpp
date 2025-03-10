#include "ACU_SystemTasks.h"

bool initialize_all_systems() {
    // Initialize the ACU Controller
    ACUControllerInstance<NUM_CELLS>::create();
    ACUControllerInstance<NUM_CELLS>::instance().init(sys_time::hal_millis());
    return true;
}

etl::delegate<bool()> mock_hv_over_threshold = etl::delegate<bool()>::create([]() -> bool {
    return true;
});
