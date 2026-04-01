#include "gtest/gtest.h"
#include <array>
#include <stddef.h>

#include <etl/delegate.h>

#include "ACUStateMachine.h"

bool request_charge_state;
bool has_bms_fault_var;
bool has_imd_fault_var;
bool received_valid_shdn_out_var;
bool is_contactor_welded;

bool charging_en = false;
bool watchdog_en = true;
bool n_latch_en = true;

etl::delegate<bool()> charge_state_requested = etl::delegate<bool()>::create([]() -> bool {
    return request_charge_state;
});

etl::delegate<bool()> has_bms_fault = etl::delegate<bool()>::create([]() -> bool {
    return has_bms_fault_var;
});

etl::delegate<bool()> has_imd_fault = etl::delegate<bool()>::create([]() -> bool {
    return has_imd_fault_var;
});

etl::delegate<bool()> contactor_welded = etl::delegate<bool()>::create([]() -> bool {
    return is_contactor_welded;
});

etl::delegate<void()> set_sw_not_ok_pin_high = etl::delegate<void()>::create([]() -> void {
});

etl::delegate<void()> set_sw_not_ok_pin_low = etl::delegate<void()>::create([]() -> void {
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

etl::delegate<void()> reset_latch = etl::delegate<void()>::create([]() -> void {
    n_latch_en = true;
});

etl::delegate<void()> disable_n_latch_en = etl::delegate<void()>::create([]() -> void {
    n_latch_en = false;
});

ACUStateMachine state_machine = ACUStateMachine(
    charge_state_requested,
    has_bms_fault,
    has_imd_fault,
    contactor_welded,
    set_sw_not_ok_pin_high,
    set_sw_not_ok_pin_low,
    received_valid_shdn_out,
    enable_cell_balancing,
    disable_cell_balancing,
    disable_watchdog,
    reinitialize_watchdog,
    reset_latch,
    disable_n_latch_en,
    0
);

TEST (ACUStateMachineTesting, initial_state) {
    request_charge_state = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = false;
    is_contactor_welded = false;

    ASSERT_EQ(state_machine.get_state(), ACUState_e::STARTUP); // initial
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::STARTUP);
    received_valid_shdn_out_var = true;

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::WELDCHECK);

    state_machine.tick_state_machine(1000);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
}   

TEST (ACUStateMachineTesting, charge_state) {
    request_charge_state = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = false;
    is_contactor_welded = false;

    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE); // initial

    request_charge_state = true;
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
    ASSERT_EQ(charging_en, false);

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::CHARGING);
    ASSERT_EQ(charging_en, true);

    request_charge_state = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::ACTIVE);
    ASSERT_EQ(charging_en, false);
}   

TEST (ACUStateMachineTesting, fault_states) {
    request_charge_state = false;
    has_bms_fault_var = false;
    has_imd_fault_var = false;
    received_valid_shdn_out_var = true;
    is_contactor_welded = false;

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
    
    is_contactor_welded = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::WELDCHECK);

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::WELDCHECK);

    state_machine.tick_state_machine(1010);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::WELDED);

    is_contactor_welded = false;
    state_machine.tick_state_machine(2020);
    ASSERT_EQ(state_machine.get_state(), ACUState_e::WELDED);

}   


