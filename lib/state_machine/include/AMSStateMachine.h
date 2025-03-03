#ifndef __AMS_STATE_MACHINE_H__
#define __AMS_STATE_MACHINE_H__

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

#include "ACUController.h"

enum class ACUState_e
{
    STARTUP = 0, 
    DRIVING = 1, 
    CHARGING = 2, 
    FAULTED = 3, 
};

class AMSStateMachine
{
public:
    AMSStateMachine() {};

    /**
     * This tick() function handles all the update logic for traversing states, and calls the functions
     * of other classes as necessary.
     * @pre Other systems are updated properly
     * @pre All relevant data exists in the data structs (VCRInterfaceData, VCRSystemData, etc.)
     * @param current_millis The system time, in millis. Passed in by the scheduler.
     * @param system_data A reference to the global system data struct.
     */
    void tick_state_machine(unsigned long current_millis, const ACUCoreData_s &system_data);

    /**
     * @return current ACU state
    */
    ACUState_e get_state();
private:
    void _set_state();

    /**
     * The function run upon the entry of the car into a new state.
     * @param new_state The state in which we are entering.
     */
    void handle_entry_logic_(ACUState_e new_state, unsigned long curr_millis);
    
    /**
     * The function run upon the exit of a state.
     * @param prev_state the state in which we are leaving.
     */
    void handle_exit_logic_(ACUState_e prev_state, unsigned long curr_millis);


    ACUState_e _current_state;
};

#endif
