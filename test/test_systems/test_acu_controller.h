#include "gtest/gtest.h"
#include "SharedFirmwareTypes.h"
#include "ACU_Globals.h"
#include "ACUController.h"

constexpr size_t num_chips = 1; 
ACUController controller = ACUControllerInstance<num_chips>::instance();
bool charging_enabled;

TEST (ACUControllerTesting, initial_state) {
    charging_enabled = false;
    ASSERT_EQ(controller.evaluate_accumulator(0, charging_enabled, {0}), ACUControllerData_s<num_chips>{});
    ASSERT_EQ(controller.evaluate_accumulator(0, charging_enabled, {0}).has_fault, false);
    ASSERT_EQ(controller.evaluate_accumulator(0, charging_enabled, {0}).charging_enabled, charging_enabled);
    ASSERT_EQ(controller.evaluate_accumulator(0, charging_enabled, {0}).cell_balance_statuses, std::array<uint16_t, num_chips>{0});
}

TEST (ACUControllerTesting, charging_state) {
    
}

TEST (ACUControllerTesting, faulted_state) {
    
}