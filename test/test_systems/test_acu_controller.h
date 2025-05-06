#include "gtest/gtest.h"
#include <array>
#include <stddef.h>

#include "ACUController.h"
#include "shared_types.h"

constexpr size_t num_cells = 12;
constexpr size_t num_chips = 1;
constexpr size_t num_cell_temps = 4;
constexpr size_t num_board_temps = 1;
bool charging_enabled;

TEST (ACUControllerTesting, initial_state) {
    ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::create();
    ACUController controller = ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::instance();

    charging_enabled = false;
    std::array<bool, num_cells> cb = {0};
    uint32_t start_time = 0;

    auto status = controller.evaluate_accumulator(start_time, {0});

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balancing_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.last_time_ov_fault_not_present, 0);
    ASSERT_EQ(status.last_time_uv_fault_not_present, 0);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, 0);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, 0);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, 0);
}

TEST (ACUControllerTesting, charging_state) {
    ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::create();
    ACUController controller = ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::instance();

    charging_enabled = true;
    std::array<bool, num_cells> cb = {0,1,0,0,1,0,0,0,1,1,0,0}; // 0b001100010010
    const uint32_t init_time = 2450;
    const uint32_t start_time = 2500;
    
    controller.init(init_time, 430.0);

    ACUData_s<num_cells, num_cell_temps, num_board_temps> data= {
        3.21, // min cell v
        3.53, // max cell v
        430.00, // pack v
        0.0, // avg v - don't care
        {3.23, 3.24, 3.23, 3.21, 3.3, 3.22, 3.21, 3.23, 3.26, 3.27, 3.22, 3.22} // individual cell voltage data
    };
    data.charging_enabled = charging_enabled;

    auto status = controller.evaluate_accumulator(init_time, data);

    ASSERT_NEAR(data.min_cell_voltage, 3.21, 0.0001);

    status = controller.evaluate_accumulator(start_time, data);

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balancing_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, true);

    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}

TEST (ACUControllerTesting, faulted_state) {
    ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::create();
    ACUController controller = ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::instance();

    charging_enabled = false; // or true doesn't matter
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3000;
    
    controller.init(init_time, 430.0);

    ACUData_s<num_cells, num_cell_temps, num_board_temps> data= {
        3.03, // min cell v
        4.21, // max cell v
        430.00, // pack v
        0.0, // avg v - don't care
        {3.18, 3.2, 3.03, 3.21, 3.10, 3.12, 3.11, 3.12, 3.18, 3.19, 4.21, 3.06}, // individual cell voltage data
        70, // cell temp c
        50, // board temp c
    };

    data.charging_enabled = charging_enabled;

    auto status = controller.evaluate_accumulator(init_time, data);

    ASSERT_NEAR(data.min_cell_voltage, 3.03, 0.0001);

    status = controller.evaluate_accumulator(start_time, data);

    ASSERT_EQ(status.has_fault, true);
    ASSERT_EQ(status.cell_balancing_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time); // because they're past thresh...
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time); 
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, init_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time); // when not charging, we don't have a fault if board temp is sub 60C
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time); // but since this, 430 > 420, then it keeps start_time
}