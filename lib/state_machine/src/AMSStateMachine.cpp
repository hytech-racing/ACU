#include "AMSStateMachine.h"

void AMSStateMachine::tick_state_machine() {
    switch(_current_state) {
        case ACUState_e::STARTUP: 
        {
            break;
        }

        case ACUState_e::DRIVING: 
        {
            break;
        }
        case ACUState_e::CHARGING: 
        {
            break;
        }
        case ACUState_e::FAULTED: 
        {
            break;
        }
        default:
            break;
    }
}