#include "gtest/gtest.h"
#include <array>
#include <stddef.h>

#include "ACUController.h"
#include "ACU_Constants.h"
#include "shared_types.h"

constexpr size_t num_cells = 12; // local test convenience
bool charging_enabled;

constexpr float ZERO_PACK_CURRENT = 0.0f; // No current flow (idle state)

ACUControllerThresholds_s thresholds = {ACUSystems::MIN_DISCHARGE_VOLTAGE_THRESH,
                                        ACUSystems::CELL_OVERVOLTAGE_THRESH,
                                        ACUSystems::CELL_UNDERVOLTAGE_THRESH,
                                        ACUSystems::CHARGING_OT_THRESH,
                                        ACUSystems::RUNNING_OT_THRESH,
                                        ACUSystems::MIN_PACK_TOTAL_VOLTAGE,
                                        ACUSystems::VOLTAGE_DIFF_TO_INIT_CB,
                                        ACUSystems::BALANCE_TEMP_LIMIT_C,
                                        ACUSystems::BALANCE_ENABLE_TEMP_THRESH_C
                                        };

TEST(ACUControllerTesting, initial_state)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();
    charging_enabled = false;
    uint32_t start_time = 0;

    controller.init(start_time, 420.0f);

    BMSCoreData_s data{}; // zeros
    auto status = controller.evaluate_accumulator(start_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.last_time_ov_fault_not_present, 0);
    ASSERT_EQ(status.last_time_uv_fault_not_present, 0);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, 0);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, 0);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, 0);
}

TEST(ACUControllerTesting, charging_state)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = true;
    std::array<bool, num_cells> cb = {0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0}; // 0b001100010010
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    // Core data used by accumulator (no per-cell array in BMSCoreData_s)
    BMSCoreData_s data = {
        3.7f,     // min cell v
        3.85f,    // max cell v
        430.0f,   // pack v

    };
    // Per-cell voltages for balancing calc
    std::array<volt, num_cells> cell_voltages = {3.7f, 3.81f, 3.7f, 3.7f, 3.81f, 3.76f, 3.7f, 3.7f, 3.84f, 3.85f, 3.75f, 3.75f};

    controller.enableCharging();

    auto status = controller.evaluate_accumulator(init_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    ASSERT_NEAR(data.min_cell_voltage, 3.7, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.charging_enabled, true);

    // Balance calculation moved out of evaluate_accumulator; verify outputs explicitly
    std::array<bool, num_cells> calc_cb{};
    controller.calculate_cell_balance_statuses(calc_cb.data(), cell_voltages.data(), num_cells, data.min_cell_voltage);
    ASSERT_EQ(calc_cb, cb);

    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}

TEST(ACUControllerTesting, faulted_state)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = false; // or true doesn't matter
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    BMSCoreData_s data= {
        3.03f, // min cell v
        4.21f, // max cell v
        430.0f, // pack v
        70.0f,  // max cell temp c
        20.0f,  // min cell temp c
        50.0f   // board temp c
    };

    auto status = controller.evaluate_accumulator(init_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    ASSERT_NEAR(data.min_cell_voltage, 3.03, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    ASSERT_EQ(status.has_fault, true);
    ASSERT_EQ(status.charging_enabled, false);

    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time); // because they're past thresh...
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, init_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time); // when not charging, we don't have a fault if board temp is sub 60C
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);  // but since this, 430 > 420, then it keeps start_time
}

TEST(ACUControllerTesting, ir_compensation_discharge)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = false;
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    // Test case: High discharge current (-100A) with voltage AT/BELOW UV threshold
    // This triggers the IR compensation check: if (input_state.min_cell_voltage <= _parameters.cell_undervoltage_thresh_v)
    // min_cell_voltage = 3.05V (AT UV threshold of 3.05V)
    // Without IR comp: would fault immediately
    // With IR comp: discharge_current = 100A, CELL_INTERNAL_RESISTANCE = 0.246/12 = 0.0205Ω
    // Internal V = 3.05V + (0.0205Ω × 100A) = 3.05V + 2.05V = 5.10V (should NOT fault)
    BMSCoreData_s data = {
        3.05f,  // min cell v - EXACTLY at UV threshold to trigger IR compensation
        4.15f,  // max cell v - well below OV threshold
        430.0f, // pack v
        40.0f,  // cell temp c - normal
        20.0f,  // min cell temp c
        35.0f   // board temp c - normal
    };

    auto status = controller.evaluate_accumulator(init_time, data, 0, -100.0f, num_cells); // -100A = discharge

    ASSERT_NEAR(data.min_cell_voltage, 3.05, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0, -100.0f, num_cells);

    // Should NOT fault - IR compensation should prevent false UV fault during high discharge
    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.charging_enabled, false);

    // All timestamps should be updated (no faults)
    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time); // Key assertion: UV not faulted due to IR comp
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}

TEST(ACUControllerTesting, ir_compensation_charge)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = false;             // Disable balancing to focus on IR compensation test
    std::array<bool, num_cells> cb = {0}; // No balancing
    const uint32_t init_time = 2450;
    const uint32_t start_time = 3500;

    controller.init(init_time, 430.0);

    // Test case: Charging current (+10A) with voltage AT/ABOVE OV threshold
    // This triggers the IR compensation check: if (input_state.max_cell_voltage >= _parameters.cell_overvoltage_thresh_v)
    // max_cell_voltage = 4.2V (EXACTLY at OV threshold of 4.2V)
    // Without IR comp: would fault immediately
    // With IR comp: discharge_current = -10A, CELL_INTERNAL_RESISTANCE = 0.246/12 = 0.0205Ω
    // Internal V = 4.2V + (0.0205Ω × -10A) = 4.2V - 0.205V = 3.995V (should NOT fault)
    BMSCoreData_s data = {
        4.10f,  // min cell v - normal
        4.20f,  // max cell v - EXACTLY at OV threshold to trigger IR compensation
        500.0f, // pack v - higher during charge
        40.0f,  // cell temp c - normal
        20.0f,  // min cell temp c
        35.0f   // board temp c - normal
    };

    auto status = controller.evaluate_accumulator(init_time, data, 0, 10.0f, num_cells); // +10A = charge

    ASSERT_NEAR(data.max_cell_voltage, 4.2, 0.0001);

    status = controller.evaluate_accumulator(start_time, data, 0, 10.0f, num_cells);

    // Should NOT fault - IR compensation should prevent false OV fault during charging
    ASSERT_EQ(status.has_fault, false);
    ASSERT_EQ(status.charging_enabled, false);

    // All timestamps should be updated (no faults)
    ASSERT_EQ(status.last_time_ov_fault_not_present, start_time); // Key assertion: OV not faulted due to IR comp
    ASSERT_EQ(status.last_time_uv_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_cell_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_board_ot_fault_not_present, start_time);
    ASSERT_EQ(status.last_time_pack_uv_fault_not_present, start_time);
}

// Tests that OV faults require 1000ms persistence before triggering
TEST(ACUControllerTesting, cell_overvoltage_fault_persistence)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = false;
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 1000;
    const uint32_t before_fault_time = 1500; // 500ms after init - should NOT fault yet
    const uint32_t after_fault_time = 2100;  // 1100ms after init - should fault (> 1000ms threshold)

    controller.init(init_time, 430.0);

    // Cell voltage JUST above OV threshold (4.21V > 4.2V), zero current
    BMSCoreData_s data = {
        3.70,                                                                     // min cell v - normal
        4.21,                                                                     // max cell v - ABOVE OV threshold
        500.00,                                                                   // pack v - normal
        40,                                                                       // cell temp c - normal
        35,                                                                       // board temp c - normal
    };

    // First evaluation at init_time - establishes OV condition
    auto status = controller.evaluate_accumulator(init_time, data, 0, ZERO_PACK_CURRENT, 126);

    // OV detected, timestamp NOT updated (still at init_time)
    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time);
    ASSERT_EQ(status.has_fault, false); // Should NOT fault yet (duration = 0ms < 1000ms)

    // Second evaluation at 500ms - still within threshold
    status = controller.evaluate_accumulator(before_fault_time, data, 0, ZERO_PACK_CURRENT, 126);
    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time); // Still stuck at init_time
    ASSERT_EQ(status.has_fault, false);                          // 500ms < 1000ms - NO fault yet

    // Third evaluation at 1100ms - exceeds threshold
    status = controller.evaluate_accumulator(after_fault_time, data, 0, ZERO_PACK_CURRENT, 126);
    ASSERT_EQ(status.last_time_ov_fault_not_present, init_time); // Still stuck at init_time
    ASSERT_EQ(status.has_fault, true);                           // 1100ms > 1000ms - FAULT!
}

// Tests that UV faults require 1000ms persistence before triggering
TEST(ACUControllerTesting, cell_undervoltage_fault_persistence)
{
    ACUControllerInstance::create(thresholds);
    ACUController controller = ACUControllerInstance::instance();

    charging_enabled = false;
    std::array<bool, num_cells> cb = {0};
    const uint32_t init_time = 1000;
    const uint32_t before_fault_time = 1500; // 500ms after init - should NOT fault yet
    const uint32_t after_fault_time = 2100;  // 1100ms after init - should fault (> 1000ms threshold)

    controller.init(init_time, 430.0);

    // Cell voltage JUST below UV threshold (3.04V < 3.05V), zero current
    BMSCoreData_s data = {
        3.04f,  // min cell v - BELOW UV threshold
        4.10f,  // max cell v - normal
        380.0f, // pack v - normal
        40.0f,  // cell temp c - normal
        20.0f,  // min cell temp c
        35.0f   // board temp c - normal
    };

    // First evaluation at init_time - establishes UV condition
    auto status = controller.evaluate_accumulator(init_time, data, 0, ZERO_PACK_CURRENT, num_cells);

    // UV detected, timestamp NOT updated (still at init_time)
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time);
    ASSERT_EQ(status.has_fault, false); // Should NOT fault yet (duration = 0ms < 1000ms)

    // Second evaluation at 500ms - still within threshold
    status = controller.evaluate_accumulator(before_fault_time, data, 0, ZERO_PACK_CURRENT, num_cells);
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time); // Still stuck at init_time
    ASSERT_EQ(status.has_fault, false);                          // 500ms < 1000ms - NO fault yet

    // Third evaluation at 1100ms - exceeds threshold
    status = controller.evaluate_accumulator(after_fault_time, data, 0, ZERO_PACK_CURRENT, num_cells);
    ASSERT_EQ(status.last_time_uv_fault_not_present, init_time); // Still stuck at init_time
    ASSERT_EQ(status.has_fault, true);                           // 1100ms > 1000ms - FAULT!
}