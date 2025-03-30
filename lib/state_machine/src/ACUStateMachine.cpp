#include "ACUStateMachine.h"

void ACUStateMachine::tick_state_machine(unsigned long current_millis) {
    switch(_current_state) {
        case ACUState_e::STARTUP: 
        {   
            if (_received_valid_shdn_out()) {
                _set_state(ACUState_e::ACTIVE, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::ACTIVE: 
        {
            if (_charge_state_requested()) {
                _set_state(ACUState_e::CHARGING, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::CHARGING: 
        {   
            if (!_charge_state_requested()) {
                _set_state(ACUState_e::ACTIVE, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::FAULTED: 
        {
            if (_received_valid_shdn_out() && !(_has_bms_fault() || _has_imd_fault())) {
                _set_state(ACUState_e::STARTUP, current_millis);
                break;
            }
            break;
        }
        default: 
        {
            break;
        }   
    }
}

void ACUStateMachine::_set_state(ACUState_e new_state, unsigned long curr_millis)
{
    _handle_exit_logic(_current_state, curr_millis);
    _current_state = new_state;
    _handle_entry_logic(_current_state, curr_millis);
}

void ACUStateMachine::_handle_exit_logic(ACUState_e prev_state, unsigned long curr_millis)
{
    switch(prev_state) {
        case ACUState_e::STARTUP: 
        {
            break;
        }
        case ACUState_e::ACTIVE: 
        {
            break;
        }
        case ACUState_e::CHARGING: 
        {
            _disable_cell_balancing();
            _disable_send_discharging_CAN_msg();
            break;
        }
        case ACUState_e::FAULTED: 
        {
            _reset_latch();
            _reinitialize_watchdog();
            break;
        }
        default:
            break;
    }
}

void ACUStateMachine::_handle_entry_logic(ACUState_e new_state, unsigned long curr_millis)
{
    switch(new_state) {
        case ACUState_e::STARTUP: 
        {
            _reinitialize_watchdog();
            break;
        }
        case ACUState_e::ACTIVE: 
        {   
            break;
        }
        case ACUState_e::CHARGING: 
        {   
            _enable_send_discharging_CAN_msg();
            _enable_cell_balancing();
            break;
        }
        case ACUState_e::FAULTED: 
        {   
            _disable_watchdog();
            _disable_n_latch_en();
            break;
        }
        default:
            break;
    }
}