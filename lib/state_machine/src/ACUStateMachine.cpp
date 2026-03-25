#include "ACUStateMachine.h"
#include <iostream>

using namespace std;


void ACUStateMachine::tick_state_machine(unsigned long current_millis) 
{
    switch(_current_state) 
    {
        case ACUState_e::STARTUP: 
        {   
            if (_received_valid_shdn_out())
            {
                _set_state(ACUState_e::WELDCHECK, current_millis);
                break;
            }

            break;
        }
        case ACUState_e::WELDCHECK:
        {
            if (current_millis - _last_state_changed_time > 500)
            {
                if (_contactor_welded()) 
                {
                    _set_state(ACUState_e::WELDED, current_millis);
                    break;
                }
                else 
                {
                    _set_state(ACUState_e::ACTIVE, current_millis);
                    break;
                }
            }
            
            break;
        }
        case ACUState_e::WELDED:
        {
            break;
        }
        
        case ACUState_e::ACTIVE: 
        {
            if (_charge_state_requested()) 
            {
                _set_state(ACUState_e::CHARGING, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) 
            {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::CHARGING: 
        {   
            if (!_charge_state_requested()) 
            {
                _set_state(ACUState_e::ACTIVE, current_millis);
                break;
            }
            if (_has_bms_fault() || _has_imd_fault()) 
            {
                _set_state(ACUState_e::FAULTED, current_millis);
                break;
            }
            if (!_received_valid_shdn_out()) 
            {
                _set_state(ACUState_e::STARTUP, current_millis);
                break;
            }
            break;
        }
        case ACUState_e::FAULTED: 
        {   

            if ((current_millis - _last_state_changed_time > 1000) && !(_has_bms_fault())) 
            {
                _reinitialize_watchdog();
            }

            if (_received_valid_shdn_out() && !(_has_bms_fault() || _has_imd_fault())) 
            {
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

    // update any time there is a state change
    _last_state_changed_time = curr_millis;
}

void ACUStateMachine::_handle_exit_logic(ACUState_e prev_state, unsigned long curr_millis)
{
    switch(prev_state) 
    {
        case ACUState_e::CHARGING: 
        {
            _disable_cell_balancing();
            break;
        }
        case ACUState_e::FAULTED: 
        {
            _set_n_latch_en_high();
            _reinitialize_watchdog();
            break;
        }
        case ACUState_e::STARTUP:
        case ACUState_e::ACTIVE:
        case ACUState_e::WELDCHECK:
        case ACUState_e::WELDED:
        default:
            break;
    }
}

void ACUStateMachine::_handle_entry_logic(ACUState_e new_state, unsigned long curr_millis)
{
    switch(new_state) 
    {
        case ACUState_e::STARTUP: 
        {
            _reinitialize_watchdog();
            break;
        }
        case ACUState_e::WELDCHECK:
        {
            break;
        }
        case ACUState_e::WELDED:
        {
            _set_sw_not_ok_pin_high();
            break;
        }
        case ACUState_e::CHARGING: 
        {   
            _enable_cell_balancing();
            break;
        }
        case ACUState_e::FAULTED: 
        {   
            _disable_watchdog();
            _set_n_latch_en_low();
            _last_state_changed_time = curr_millis;
            break;
        }
        case ACUState_e::ACTIVE:
        {
            _set_sw_not_ok_pin_low();
            break;
        }
        default:
            break;
    }
}

