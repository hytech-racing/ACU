#ifndef __ACU_MESSAGE_INTERFACE_H__
#define __ACU_MESSAGE_INTERFACE_H__

/* Library Icludes*/
#include "hytech_msgs.pb.h"
#include <optional>
#include <Arduino.h>

using volt = float;
using celcius = float;

template<size_t num_chips, size_t num_humidity_sensors, size_t num_board_thermistors>
struct hytech_msgs_ACUOutputData {
    std::array<std::array<std::optional<volt>, 12>, num_chips> voltages;
    std::array<celcius, 4 * num_chips> cell_temperatures;
    std::array<float, num_humidity_sensors> humidity;
    std::array<float, num_board_thermistors> board_temperatures;
    float min_voltage;
    float max_voltage;
    size_t min_voltage_cell_id;              // 0 - 125
    size_t max_voltage_cell_id;              // 0 - 125
    size_t max_board_temperature_segment_id; // 0 - 5
    size_t max_humidity_segment_id;          // 0 - 5
    size_t max_cell_temperature_cell_id;     // 0 - 47
    float total_voltage;
    float average_cell_temperature;
};

typedef struct _hytech_msgs_ACUCommandData { 
    int64_t prev_ACU_recv_millis;
    // wtv
} hytech_msgs_ACUCommandData;

struct ACUData_s
{
    /// @brief the latest time that the ACU received a message w.r.t the current tick's millis
    int64_t last_receive_time_millis = -1;
};


template <size_t num_chips>
class ACUEthernetInterface 
{
public:
    using ACUOutputData = hytech_msgs_ACUOutputData<num_chips, (num_chips + 1) / 2, (num_chips + 1) / 2>;

    ACUEthernetInterface() {
        _latest_data.last_receive_time_millis = -1;
        // Additional Initialization
    };
    void receive_pb_msg(const hytech_msgs_ACUCommandData &msg_in, unsigned long curr_millis);
    ACUOutputData make_acu_msg();
    ACUData_s get_latest_data() { return _latest_data; }

private: 
    ACUData_s _latest_data = {};
};
#include <ACUEthernetInterface.tpp>

#endif