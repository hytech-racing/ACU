#include "gtest/gtest.h"

#include <array>
#include <stddef.h>

#include "SharedFirmwareTypes.h"
#include "ACU_Globals.h"
#include "ACUController.h"

constexpr size_t num_chips = 1; 
ACUController controller = ACUControllerInstance<num_chips>::instance();
bool charging_enabled;

TEST (ACUControllerTesting, initial_state) {
    charging_enabled = false;
    std::array<uint16_t, num_chips> cb = {0};
    uint32_t start_time = 0;

    auto status = controller.evaluate_accumulator(start_time, charging_enabled, {0});

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balance_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.ov_start_time, 0);
    ASSERT_EQ(status.uv_start_time, 0);
    ASSERT_EQ(status.cell_ot_start_time, 0);
    ASSERT_EQ(status.board_ot_start_time, 0);
    ASSERT_EQ(status.pack_uv_start_time, 0);
}

TEST (ACUControllerTesting, charging_state) {
     charging_enabled = true;
    std::array<uint16_t, num_chips> cb = {0};
    uint32_t start_time = 0;

    auto status = controller.evaluate_accumulator(start_time, charging_enabled, {0});

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balance_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.ov_start_time, 0);
    ASSERT_EQ(status.uv_start_time, 0);
    ASSERT_EQ(status.cell_ot_start_time, 0);
    ASSERT_EQ(status.board_ot_start_time, 0);
    ASSERT_EQ(status.pack_uv_start_time, 0);
    const uint32_t time_arb = 2500;
}

TEST (ACUControllerTesting, faulted_state) {
    
}