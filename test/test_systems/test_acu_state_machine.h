#include "gtest/gtest.h"
#include <array>
#include <stddef.h>

#include <etl/delegate.h>

#include "ACUStateMachine.h"

bool received_CCU_msg_var;
bool has_bms_fault_var;
bool has_imd_fault_var;
bool received_valid_shdn_out_var;

bool charging_en = false;
bool watchdog_en = true;
bool n_latch_en = true;

etl::delegate<bool()> received_CCU_message = etl::delegate<bool()>::create([]() -> bool {
    return received_CCU_msg_var;
});

etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool {
    return has_bms_fault_var;
});

etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool {
    return has_imd_fault_var;
});

etl::delegate<bool()> received_valid_shdn_out = etl::delegate<bool()>::create([]() -> bool {
    return received_valid_shdn_out_var;
});

etl::delegate<void()> enable_cell_balancing = etl::delegate<void()>::create([]() -> void {
    charging_en = true;
});

etl::delegate<void()> disable_cell_balancing = etl::delegate<void()>::create([]() -> void {
    charging_en = false;
});

etl::delegate<void()> disable_watchdog = etl::delegate<void()>::create([]() -> void {
    watchdog_en = false;
});

etl::delegate<void()> reinitialize_watchdog = etl::delegate<void()>::create([]() -> void {
    watchdog_en = true;
});

etl::delegate<void()> disable_n_latch_en = etl::delegate<void()>::create([]() -> void {
    n_latch_en = false;
});

etl::delegate<void()> reset_latch = etl::delegate<void()>::create([]() -> void {
    n_latch_en = true;
});

ACUStateMachine state_machine = ACUStateMachine(
    received_CCU_message,
    has_bms_fault,
    has_imd_fault,
    received_valid_shdn_out,
    enable_cell_balancing,
    disable_cell_balancing,
    disable_watchdog,
    reinitialize_watchdog,
    disable_n_latch_en,
    reset_latch
);

TEST (ACUStateMachineTesting, initial_state) {
    received_CCU_msg_var = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = false;

    ASSERT_EQ(state_machine.get_state(), ACUState_e::STARTUP); // initial
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::STARTUP);

    received_valid_shdn_out_var = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
}   

TEST (ACUStateMachineTesting, CCU_msg_state) {
    received_CCU_msg_var = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = false;

    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE); // initial

    received_CCU_msg_var = true;
    ASSERT_EQ(charging_en, false);

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::CHARGING);
    ASSERT_EQ(charging_en, true);

    received_CCU_msg_var = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
    ASSERT_EQ(charging_en, false);
}   

TEST (ACUStateMachineTesting, fault_states) {
    received_CCU_msg_var = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = true;

    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE); // initial

    has_bms_fault_var = true;

    ASSERT_EQ(n_latch_en, true); 
    ASSERT_EQ(watchdog_en, true);

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::FAULTED);
    ASSERT_EQ(n_latch_en, false);
    ASSERT_EQ(watchdog_en, false);
    ASSERT_EQ(charging_en, false);

    received_valid_shdn_out_var = false; 
    has_imd_fault_var = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::FAULTED);

    has_bms_fault_var = has_imd_fault_var = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::FAULTED);

    has_bms_fault_var = true;
    received_valid_shdn_out_var = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::FAULTED);

    has_bms_fault_var = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::STARTUP);

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
}   


