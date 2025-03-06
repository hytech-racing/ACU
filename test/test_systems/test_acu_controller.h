#include "gtest/gtest.h"

#include <array>
#include <stddef.h>

#include "ACUController.h"

constexpr size_t num_chips = 1;
bool charging_enabled;

TEST (ACUControllerTesting, initial_state) {
    ACUControllerInstance<num_chips>::create();
    ACUController controller = ACUControllerInstance<num_chips>::instance();

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
    ACUControllerInstance<num_chips>::create();
    ACUController controller = ACUControllerInstance<num_chips>::instance();

    charging_enabled = true;
    std::array<uint16_t, num_chips> cb = {0b001100010010};
    const uint32_t init_time = 2450;
    const uint32_t start_time = 2500;
    
    controller.init(init_time);

    ACUData_s<num_chips> data= {
        3.21, // min cell v
        3.53, // max cell v
        430.00, // pack v
        {3.23, 3.24, 3.23, 3.21, 3.3, 3.22, 3.21, 3.23, 3.26, 3.27, 3.22, 3.22} // individual cell voltage data
    };

    auto status = controller.evaluate_accumulator(init_time, charging_enabled, data);

    ASSERT_NEAR(data.min_cell_voltage, 3.21, 0.0001);
    ASSERT_EQ(controller._acu_state.ov_start_time, 2450);

    status = controller.evaluate_accumulator(start_time,charging_enabled, data);

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balance_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, true);

    ASSERT_EQ(status.ov_start_time, start_time);
    ASSERT_EQ(status.uv_start_time, start_time);
    ASSERT_EQ(status.cell_ot_start_time, start_time);
    ASSERT_EQ(status.board_ot_start_time, start_time);
    ASSERT_EQ(status.pack_uv_start_time, start_time);
}

TEST (ACUControllerTesting, faulted_state) {
    
}