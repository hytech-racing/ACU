#ifndef __CCU_INTERFACE_H__
#define __CCU_INTERFACE_H__

#include <cstdint>
#include <tuple>
#include <utility>

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "FlexCAN_T4.h"
#include "SharedFirmwareTypes.h"

struct CCUCANInterfaceData_s {
    StampedPedalsSystemData_s stamped_pedals;
    DashInputState_s dash_input_state;
};

class CCUInterface {
public:

    CCUInterface() = delete;

    CCUInterface(unsigned long init_millis, unsigned long max_heartbeat_interval_ms) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_recv_millis = 0;
        _curr_data.stamped_pedals.heartbeat_ok = false; // start out false
    };

    bool is_start_button_pressed() { return _curr_data.dash_input_state.start_btn_is_pressed; }
    bool is_brake_pressed() {return _curr_data.stamped_pedals.pedals_data.brake_is_pressed; }
    bool is_pedals_heartbeat_not_ok() {return !_curr_data.stamped_pedals.heartbeat_ok; }
    void reset_pedals_heartbeat();
    
    void receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis);
    void receive__message(const CAN_message_t& msg, unsigned long curr_millis);
    
    CCUCANInterfaceData_s get_latest_data();

    void send_buzzer_start_message();

private:

    CCUCANInterfaceData_s _curr_data;

    unsigned long _max_heartbeat_interval_ms;
    bool _first_received_message_heartbeat_init = false;
    
};

using CCUInterfaceInstance = etl::singleton<CCUInterface>;


#endif