#include "ACUStateMachine.h"

void ACUStateMachine::tick_state_machine(unsigned long current_millis) {
    switch(_current_state) {
        case ACUState_e::SHDN_OUT_VOLTAGE_LOW: 
        {   
            if (_received_valid_shdn_out()) {
                _set_state(ACUState_e::SHDN_OUT_VOLTAGE_HIGH, current_millis);
                break;
            }
            if ((current_millis - _last_state_changed_time > 2000) && (_has_bms_fault() || _has_imd_fault())) {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::SHDN_OUT_VOLTAGE_HIGH: 
        {
            if (_balancing_requested() && _received_valid_shdn_out()) {
                _set_state(ACUState_e::REQUEST_CELL_BALANCING, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::REQUEST_CELL_BALANCING: 
        {   
            if (!_balancing_requested()) {
                _set_state(ACUState_e::SHDN_OUT_VOLTAGE_HIGH, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            if (!_received_valid_shdn_out()) {
                _set_state(ACUState_e::SHDN_OUT_VOLTAGE_LOW, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::FAULTED: 
        {   
            if ((current_millis - _last_state_changed_time > 1000) && !(_has_bms_fault())) {
                _reinitialize_watchdog();
            }

            if (_received_valid_shdn_out() && !(_has_bms_fault() || _has_imd_fault())) {
                _set_state(ACUState_e::SHDN_OUT_VOLTAGE_LOW, current_millis);
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
        case ACUState_e::REQUEST_CELL_BALANCING: 
        {
            _disable_cell_balancing();
            break;
        }
        case ACUState_e::FAULTED: 
        {
            _set_n_latch_en_low();
            _reinitialize_watchdog();
            break;
        }
        case ACUState_e::SHDN_OUT_VOLTAGE_LOW:
        case ACUState_e::SHDN_OUT_VOLTAGE_HIGH:
        default:
            break;
    }
}

void ACUStateMachine::_handle_entry_logic(ACUState_e new_state, unsigned long curr_millis)
{
    switch(new_state) {
        case ACUState_e::SHDN_OUT_VOLTAGE_LOW: 
        {
            _reinitialize_watchdog();
            break;
        }
        case ACUState_e::REQUEST_CELL_BALANCING: 
        {   
            _request_cell_balancing();
            break;
        }
        case ACUState_e::FAULTED: 
        {   
            _disable_watchdog();
            _set_n_latch_en_high();
            _last_state_changed_time = curr_millis;
            break;
        }
        case ACUState_e::SHDN_OUT_VOLTAGE_HIGH:
        default:
            break;
    }
}