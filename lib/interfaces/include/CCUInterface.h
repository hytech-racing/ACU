#ifndef __CCU_INTERFACE_H__
#define __CCU_INTERFACE_H__

#include <cstdint>
#include <tuple>
#include <utility>
#include <array>

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "FlexCAN_T4.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

constexpr size_t NUM_CELLS = 126;
constexpr size_t NUM_CELLTEMPS = 48;
constexpr size_t NUM_CHIPS = 12;

struct CCUCANInterfaceData_s
{
    unsigned long last_time_charging_requested;
    bool charging_requested;
};

class CCUInterface
{
public:
    CCUInterface() = delete;

    CCUInterface(unsigned long init_millis, unsigned long charing_enable_threshold_ms) : _min_charging_enable_threshold(charing_enable_threshold_ms)
    {
        _curr_data.last_time_charging_requested = 0;
        _curr_data.charging_requested = false;
    };

    bool is_charging_requested() { return _curr_data.charging_requested; }

    void receive_CCU_status_message(const CAN_message_t &msg, unsigned long curr_millis);

    void set_system_latch_state(unsigned long curr_millis, bool is_latched);

    CCUCANInterfaceData_s get_latest_data(unsigned long curr_millis);

    template <size_t num_cells, size_t num_celltemps>
    void set_ACU_data(ACUData_s<num_cells, num_celltemps> input)
    {
        _acu_core_data.avg_cell_voltage = input.avg_cell_voltage;
        _acu_core_data.max_cell_voltage = input.max_cell_voltage;
        _acu_core_data.min_cell_voltage = input.min_cell_voltage;
        _acu_core_data.pack_voltage = input.pack_voltage;
        for (size_t c = 0; c < num_cells; c++)
        {
            _acu_all_data.cell_voltages[c] = input.voltages[c];
        }
        for (size_t temp = 0; temp < num_celltemps; temp++)
        {
            _acu_all_data.cell_temps[temp] = input.cell_temps[temp];
        }
    }

    void handle_enqueue_acu_status_CAN_message();
    void handle_enqueue_acu_core_voltages_CAN_message();
    void handle_enqueue_acu_voltages_A_CAN_message();
    void handle_enqueue_acu_voltages_B_CAN_message();

private:
    CCUCANInterfaceData_s _curr_data;
    ACUCoreData_s _acu_core_data;
    ACUAllData_s<NUM_CELLS, NUM_CELLTEMPS, NUM_CHIPS> _acu_all_data;

    unsigned long _min_charging_enable_threshold;
};

using CCUInterfaceInstance = etl::singleton<CCUInterface>;

#endif