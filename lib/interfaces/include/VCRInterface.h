#ifndef VCR_INTERFACE_H
#define VCR_INTERFACE_H

/* ETL Library */
#include "etl/delegate.h"
#include "etl/singleton.h"

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "FlexCAN_T4.h"
#include "shared_types.h"
#include <cstdint>
#include <tuple>
#include <utility>
#include <array>


struct VCRCANInterfaceData_s
{
    bool imd_ok;
    bool latch_ok;
    bool bms_ok;
    unsigned long last_time_msg_sent;
};

class VCRInterface
{
public:

    VCRInterface() = delete;

    VCRInterface(unsigned long init_millis)
    {
        _curr_data.last_time_msg_sent = init_millis;
        _curr_data.imd_ok = true;
        _curr_data.latch_ok = true;
        _curr_data.bms_ok = true;
    };

    void set_monitoring_data(bool imd_ok, bool bms_ok, bool latch_ok);

    VCRCANInterfaceData_s get_latest_data(unsigned long curr_millis);

    void handle_enqueue_acu_ok_CAN_message();

private:

    VCRCANInterfaceData_s _curr_data;

    unsigned long _min_charging_enable_threshold;
};

using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif