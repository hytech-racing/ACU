#ifndef __CCU_INTERFACE_H__
#define __CCU_INTERFACE_H__

#include <cstdint>
#include <tuple>
#include <utility>

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "FlexCAN_T4.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

struct CCUCANInterfaceData_s {
    unsigned long last_time_charging_requested;
    bool charging_requested;
};

class CCUInterface {
public:

    CCUInterface() = delete;

    CCUInterface(unsigned long init_millis, unsigned long charing_enable_threshold_ms) : _min_charging_enable_threshold(charing_enable_threshold_ms)
    {
        _curr_data.last_time_charging_requested = 0;
        _curr_data.charging_requested = false;
    };

    bool is_charging_requested() { return _curr_data.charging_requested; }
    
    void receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    CCUCANInterfaceData_s get_latest_data(unsigned long curr_millis);

    void set_ACU_core_data(ACUCoreData_s input);

    void handle_enqueue_acu_status_CAN_message();
    void handle_enqueue_acu_voltages_CAN_message();

private:

    CCUCANInterfaceData_s _curr_data;
    ACUCoreData_s _acu_core_data; 

    unsigned long _min_charging_enable_threshold;
};

using CCUInterfaceInstance = etl::singleton<CCUInterface>;


#endif