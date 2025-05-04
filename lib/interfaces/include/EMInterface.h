#ifndef __EM_INTERFACE_H__
#define __EM_INTERFACE_H__

#include <etl/singleton.h>

#include "SharedFirmwareTypes.h"
#include "FlexCAN_T4.h"

struct EMData_s 
{
    uint32_t prev_time_stamp_ms = 0;
    uint32_t time_since_prev_msg_ms = 0;
    volt em_voltage = 0;
    float em_current = 0;
};

class EMInterface
{
public:
    EMInterface() = delete;
    EMInterface(uint32_t init_millis) { _em_data.prev_time_stamp_ms = init_millis; }

    void receive_EM_measurement_message(const CAN_message_t &msg, uint32_t curr_millis);
    void receive_EM_status_message(const CAN_message_t &msg, uint32_t curr_millis);
    EMData_s get_latest_data(uint32_t curr_millis);
private:
    EMData_s _em_data;
};

using EMInterfaceInstance = etl::singleton<EMInterface>;

#endif // __CCU_INTERFACE_H__