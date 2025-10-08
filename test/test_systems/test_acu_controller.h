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

    auto status = controller.evaluate_accumulator(start_time, {0}, 0.0f);

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
    const uint32_t start_time = 3500;
    
    controller.init(init_time, 430.0);

    ACUData_s<num_cells, num_cell_temps, num_board_temps> data= {
        3.7, // min cell v
        3.85, // max cell v
        430.00, // pack v
        0.0, // avg v - don't care
        {3.7, 3.81, 3.7, 3.7, 3.81, 3.76, 3.7, 3.7, 3.84, 3.85, 3.75, 3.75} // individual cell voltage data
    };
    data.charging_enabled = charging_enabled;

    auto status = controller.evaluate_accumulator(init_time, data, 0.0f);

    ASSERT_NEAR(data.min_cell_voltage, 3.7, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0.0f);

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
    const uint32_t start_time = 3500;
    
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

    auto status = controller.evaluate_accumulator(init_time, data, 0.0f);

    ASSERT_NEAR(data.min_cell_voltage, 3.03, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0.0f);

    ASSERT_EQ(status.has_fault, true);
    ASSERT_EQ(status.cell_balancing_statuses, cb); 
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time); // because they're past thresh...
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, init_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time); // when not charging, we don't have a fault if board temp is sub 60C
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time); // but since this, 430 > 420, then it keeps start_time
}

TEST (ACUControllerTesting, ir_compensation_discharge) {
    ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::create();
    ACUController controller = ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::instance();

    charging_enabled = false;
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    // Test case: High discharge current (-100A) with voltage AT/BELOW UV threshold
    // This triggers the IR compensation check: if (input_state.min_cell_voltage <= _parameters.uv_thresh_v)
    // min_cell_voltage = 3.05V (AT UV threshold of 3.05V)
    // Without IR comp: would fault immediately
    // With IR comp: discharge_current = 100A, CELL_IR = 0.246/12 = 0.0205Ω
    // Internal V = 3.05V + (0.0205Ω × 100A) = 3.05V + 2.05V = 5.10V (should NOT fault)
    ACUData_s<num_cells, num_cell_temps, num_board_temps> data= {
        3.05, // min cell v - EXACTLY at UV threshold to trigger IR compensation
        4.15, // max cell v - well below OV threshold
        430.00, // pack v
        0.0, // avg v - don't care
        {3.18, 3.2, 3.05, 3.21, 3.10, 3.12, 3.11, 3.12, 3.18, 3.19, 4.15, 3.08}, // individual cell voltage data
        40, // cell temp c - normal
        35, // board temp c - normal
    };

    data.charging_enabled = charging_enabled;

    auto status = controller.evaluate_accumulator(init_time, data, -100.0f); // -100A = discharge

    ASSERT_NEAR(data.min_cell_voltage, 3.05, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, -100.0f);

    // Should NOT fault - IR compensation should prevent false UV fault during high discharge
    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balancing_statuses, cb);
    ASSERT_EQ(status.charging_enabled, false);

    // All timestamps should be updated (no faults)
    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time); // Key assertion: UV not faulted due to IR comp
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}

TEST (ACUControllerTesting, ir_compensation_charge) {
    ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::create();
    ACUController controller = ACUControllerInstance<num_cells, num_cell_temps, num_board_temps>::instance();

    charging_enabled = false; // Disable balancing to focus on IR compensation test
    std::array<bool, num_cells> cb = {0}; // No balancing
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    // Test case: Charging current (+10A) with voltage AT/ABOVE OV threshold
    // This triggers the IR compensation check: if (input_state.max_cell_voltage >= _parameters.ov_thresh_v)
    // max_cell_voltage = 4.2V (EXACTLY at OV threshold of 4.2V)
    // Without IR comp: would fault immediately
    // With IR comp: discharge_current = -10A, CELL_IR = 0.246/12 = 0.0205Ω
    // Internal V = 4.2V + (0.0205Ω × -10A) = 4.2V - 0.205V = 3.995V (should NOT fault)
    ACUData_s<num_cells, num_cell_temps, num_board_temps> data= {
        4.10, // min cell v - normal
        4.20, // max cell v - EXACTLY at OV threshold to trigger IR compensation
        500.00, // pack v - higher during charge
        0.0, // avg v - don't care
        {4.10, 4.15, 4.12, 4.13, 4.14, 4.11, 4.16, 4.17, 4.18, 4.20, 4.15, 4.14}, // individual cell voltage data
        40, // cell temp c - normal
        35, // board temp c - normal
    };

    data.charging_enabled = charging_enabled;

    auto status = controller.evaluate_accumulator(init_time, data, 10.0f); // +10A = charge

    ASSERT_NEAR(data.max_cell_voltage, 4.2, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 10.0f);

    // Should NOT fault - IR compensation should prevent false OV fault during charging
    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.cell_balancing_statuses, cb);
    ASSERT_EQ(status.charging_enabled, false);

    // All timestamps should be updated (no faults)
    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time); // Key assertion: OV not faulted due to IR comp
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}