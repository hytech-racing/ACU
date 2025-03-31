#ifndef __ACU_STATE_MACHINE_H__
#define __ACU_STATE_MACHINE_H__

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

#include <etl/delegate.h>
#include "etl/singleton.h"

enum class ACUState_e
{
    STARTUP = 0, 
    ACTIVE = 1, 
    CHARGING = 2, 
    FAULTED = 3, 
};

class ACUStateMachine
{
public:
    ACUStateMachine(
        etl::delegate<bool()> charge_state_requested,
        etl::delegate<bool()> has_bms_fault,
        etl::delegate<bool()> has_imd_fault,
        etl::delegate<bool()> received_valid_shdn_out,
        etl::delegate<void()> enable_cell_balancing,
        etl::delegate<void()> disable_cell_balancing,
        etl::delegate<void()> disable_watchdog,
        etl::delegate<void()> reinitialize_watchdog,
        etl::delegate<void()> disable_n_latch_en,
        etl::delegate<void()> reset_latch
    ) :
    _charge_state_requested(charge_state_requested),
    _has_bms_fault(has_bms_fault),
    _has_imd_fault(has_imd_fault),
    _received_valid_shdn_out(received_valid_shdn_out),
    _enable_cell_balancing(enable_cell_balancing),
    _disable_cell_balancing(disable_cell_balancing),
    _disable_watchdog(disable_watchdog),
    _reinitialize_watchdog(reinitialize_watchdog),
    _disable_n_latch_en(disable_n_latch_en),
    _reset_latch(reset_latch)
    {
        _current_state = ACUState_e::STARTUP;
    };

    void tick_state_machine(unsigned long current_millis);

    /**
     * @return current ACU state
    */
    ACUState_e get_state() { return _current_state; } 

private:

    void _set_state(ACUState_e new_state, unsigned long curr_millis);

    /**
     * The function run upon the entry of the car into a new state.
     * @param new_state The state in which we are entering.
     */
    void _handle_entry_logic(ACUState_e new_state, unsigned long curr_millis);
    
    /**
     * The function run upon the exit of a state.
     * @param prev_state the state in which we are leaving.
     */
    void _handle_exit_logic(ACUState_e prev_state, unsigned long curr_millis);
        
    ACUState_e _current_state;

    // Lamdas for state machine abstraction, functions defined in main
    etl::delegate<bool()> _charge_state_requested; 
    etl::delegate<bool()> _has_bms_fault;
    etl::delegate<bool()> _has_imd_fault;
    etl::delegate<bool()> _received_valid_shdn_out;
    /// @brief setters
    etl::delegate<void()> _enable_cell_balancing;
    etl::delegate<void()> _disable_cell_balancing;
    etl::delegate<void()> _disable_watchdog;
    etl::delegate<void()> _reinitialize_watchdog;
    etl::delegate<void()> _disable_n_latch_en;
    etl::delegate<void()> _reset_latch;
};

using ACUStateMachineInstance = etl::singleton<ACUStateMachine>;

#endif