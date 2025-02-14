#ifndef __AMS_STATE_MACHINE_H__
#define __AMS_STATE_MACHINE_H__


enum class ACUState_e
{
    STARTUP = 0,
    TRACTIVE_SYSTEM_NOT_ACTIVE = 1,
    TRACTIVE_SYSTEM_ACTIVE = 2,
    ENABLING_INVERTERS = 3,
    WAITING_READY_TO_DRIVE_SOUND = 4,
    READY_TO_DRIVE = 5
};

class AMSStateMachine
{
    public:
        AMSStateMachine();
        ACUState_e get_state();
    private: 
        ACUState_e _current_state;
};

#endif
