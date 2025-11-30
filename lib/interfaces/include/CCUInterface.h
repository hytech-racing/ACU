#ifndef CCU_INTERFACE_H
#define CCU_INTERFACE_H

#include <cstdint>
#include <tuple>
#include <utility>
#include <array>

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "FlexCAN_T4.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include "ACUCANBuffers.h"  // NEW: Include shared buffer declarations


namespace ccu_interface_defaults{
    constexpr const uint16_t MIN_CHARGING_ENABLE_THRESHOLD_MS = 1000;
    constexpr const size_t VOLTAGE_CELLS_PER_GROUP = 3;
    constexpr const size_t VOLTAGE_CELL_GROUPS_PER_IC_EVEN = 4;
    constexpr const size_t VOLTAGE_CELL_GROUPS_PER_IC_ODD = 3;
    constexpr const size_t TEMP_CELL_GROUPS_PER_IC = 2;
    constexpr const size_t TEMP_CELLS_PER_GROUP = 2;
};

struct CCUInterfaceParams_s {
    unsigned long min_charging_enable_threshold;
    size_t voltage_cell_groups_per_ic_even;
    size_t voltage_cell_groups_per_ic_odd;
    size_t temp_cell_groups_per_ic;
    size_t voltage_cells_per_group;
    size_t temp_cells_per_group;
};
struct CCUCANInterfaceData_s
{
    unsigned long last_time_charging_with_balancing_requested;
    unsigned long prev_ccu_msg_recv_ms;

    size_t current_voltage_group_chip_id;
    size_t current_voltage_cell_group_id;
    size_t current_voltage_cell_id;

    size_t current_temp_group_chip_id;
    size_t current_temp_group_id;
    size_t current_temp_cell_id;

    size_t current_temp_board_id;
};

class CCUInterface
{
public:
    CCUInterface() = delete;

    CCUInterface(unsigned long init_millis,
                CCUInterfaceParams_s params = {
                    .min_charging_enable_threshold = ccu_interface_defaults::MIN_CHARGING_ENABLE_THRESHOLD_MS,
                    .voltage_cell_groups_per_ic_even = ccu_interface_defaults::VOLTAGE_CELL_GROUPS_PER_IC_EVEN,
                    .voltage_cell_groups_per_ic_odd = ccu_interface_defaults::VOLTAGE_CELL_GROUPS_PER_IC_ODD,
                    .temp_cell_groups_per_ic = ccu_interface_defaults::TEMP_CELL_GROUPS_PER_IC,
                    .voltage_cells_per_group = ccu_interface_defaults::VOLTAGE_CELLS_PER_GROUP,
                    .temp_cells_per_group = ccu_interface_defaults::TEMP_CELLS_PER_GROUP
                }
            ) : _ccu_params{params}
    {

        _curr_data.last_time_charging_with_balancing_requested = 0;
        _curr_data.prev_ccu_msg_recv_ms = 0;

        _curr_data.current_voltage_cell_group_id = 0;
        _curr_data.current_voltage_group_chip_id = 0;
        _curr_data.current_voltage_cell_id = 0;

        _curr_data.current_temp_group_id = 0;
        _curr_data.current_temp_group_chip_id = 0;
        _curr_data.current_temp_cell_id = 0;


        _curr_data.current_temp_board_id = 0;
    };

    bool is_connected_to_CCU(unsigned long curr_millis) {return (curr_millis - _curr_data.prev_ccu_msg_recv_ms) < _ccu_params.min_charging_enable_threshold; }

    bool is_charging_with_balancing_requested(unsigned long curr_millis) {return (curr_millis - _curr_data.last_time_charging_with_balancing_requested) < _ccu_params.min_charging_enable_threshold; }

    void receive_CCU_status_message(const CAN_message_t &msg, unsigned long curr_millis);

    void handle_enqueue_acu_status_CAN_message(bool shdn_out_voltage_high);

    void handle_enqueue_acu_voltage_statistics_CAN_message(float max_cell_voltage, float min_cell_voltage, float pack_voltage, float avg_cell_voltage);
    void handle_enqueue_acu_cell_voltages_CAN_message(const volt* cell_voltages, const size_t* voltage_cells_per_chip, const size_t num_of_chips);
    void handle_enqueue_acu_temp_statistics_CAN_message(celsius max_board_temp, celsius max_cell_temp, celsius min_cell_temp);
    void handle_enqueue_acu_cell_temps_CAN_message(const celsius* cell_temps, const size_t* temp_cells_per_chip, const size_t num_of_chips);
    void handle_enqueue_acu_board_temps_CAN_message(const celsius* board_temps, const size_t num_of_boards);

    CCUCANInterfaceData_s get_latest_data();

private:
    CCUCANInterfaceData_s _curr_data;
    CCUInterfaceParams_s _ccu_params;

    bool _increment_and_loop_id(size_t &id, size_t max_id, size_t increment = 1)
    {
        if (id + increment >= max_id) {
            id = 0;
            return true;
        } else {
            id += increment;
            return false;
        }
    }


};

using CCUInterfaceInstance = etl::singleton<CCUInterface>;

#endif // CCU_INTERFACE_H