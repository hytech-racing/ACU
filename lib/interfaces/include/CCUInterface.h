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


enum ChargingCommand_e
{
    CHARGE = 0,
    IDLE = 1,
};
namespace ccu_interface_defaults{
    constexpr const uint16_t MIN_CHARGING_ENABLE_THRESHOLD_MS = 1000;
    constexpr const size_t NUM_CELLS = 126;
    constexpr const size_t NUM_CELLTEMPS = 48;
    constexpr const size_t NUM_CHIPS = 12;
    constexpr const size_t VOLTAGE_CELLS_PER_GROUP = 3;
    constexpr const size_t VOLTAGE_CELL_GROUPS_PER_IC_EVEN = 4;
    constexpr const size_t VOLTAGE_CELL_GROUPS_PER_IC_ODD = 3;
    constexpr const size_t TEMP_CELL_GROUPS_PER_IC = 2;
    constexpr const size_t TEMP_CELLS_PER_GROUP = 2;
};
struct CCUInterfaceParams_s {
    unsigned long min_charging_enable_threshold;
    size_t num_cells;
    size_t num_celltemps;
    size_t num_chips;
    size_t voltage_cell_groups_per_ic_even;
    size_t voltage_cell_groups_per_ic_odd;
    size_t temp_cell_groups_per_ic;
    size_t voltage_cells_per_group;
    size_t temp_cells_per_group;
};
struct CCUCANInterfaceData_s
{
    unsigned long last_time_charging_requested;
    unsigned long prev_ccu_msg_recv_ms;
    bool charging_requested;
    bool is_connected_to_CCU;
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
                    .num_cells = ccu_interface_defaults::NUM_CELLS,
                    .num_celltemps = ccu_interface_defaults::NUM_CELLTEMPS,
                    .num_chips = ccu_interface_defaults::NUM_CHIPS,
                    .voltage_cell_groups_per_ic_even = ccu_interface_defaults::VOLTAGE_CELL_GROUPS_PER_IC_EVEN,
                    .voltage_cell_groups_per_ic_odd = ccu_interface_defaults::VOLTAGE_CELL_GROUPS_PER_IC_ODD,
                    .temp_cell_groups_per_ic = ccu_interface_defaults::TEMP_CELL_GROUPS_PER_IC,
                    .voltage_cells_per_group = ccu_interface_defaults::VOLTAGE_CELLS_PER_GROUP,
                    .temp_cells_per_group = ccu_interface_defaults::TEMP_CELLS_PER_GROUP
                }
            ) : _ccu_params{params}
    {
        _curr_data.last_time_charging_requested = 0;
        _curr_data.prev_ccu_msg_recv_ms = 0;
        _curr_data.charging_requested = false;
        _curr_data.is_connected_to_CCU = false;
        _curr_data.current_voltage_cell_group_id = 0;
        _curr_data.current_voltage_group_chip_id = 0;
        _curr_data.current_voltage_cell_id = 0;
        _curr_data.current_temp_group_id = 0;
        _curr_data.current_temp_group_chip_id = 0;
        _curr_data.current_temp_cell_id = 0;
        _curr_data.current_temp_board_id = 0;
    };

    bool is_charging_requested() { return _curr_data.charging_requested; }

    bool is_connected_to_CCU() {return _curr_data.is_connected_to_CCU; }

    void receive_CCU_status_message(const CAN_message_t &msg, unsigned long curr_millis);

    void set_system_latch_state(unsigned long curr_millis, bool is_latched);

    CCUCANInterfaceData_s get_latest_data(unsigned long curr_millis);

    template <size_t num_cells, size_t num_celltemps, size_t num_chips>
    void set_ACU_data(ACUAllData_s<num_cells, num_celltemps, num_chips> input)
    {
        _acu_core_data.avg_cell_voltage = input.core_data.avg_cell_voltage;
        _acu_core_data.max_cell_voltage = input.core_data.max_cell_voltage;
        _acu_core_data.min_cell_voltage = input.core_data.min_cell_voltage;
        _acu_core_data.pack_voltage = input.core_data.pack_voltage;
        for (size_t c = 0; c < num_cells; c++)
        {
            _acu_all_data.cell_voltages[c] = input.cell_voltages[c];
        }
        for (size_t temp = 0; temp < num_celltemps; temp++)
        {
            _acu_all_data.cell_temps[temp] = input.cell_temps[temp];
        }
        for (size_t board = 0; board < num_chips; board++) {
            _acu_all_data.board_temps[board] = input.board_temps[board];
        }
        _acu_all_data.core_data.max_board_temp = input.core_data.max_board_temp;
        _acu_all_data.core_data.min_cell_temp = input.core_data.min_cell_temp;
        _acu_all_data.core_data.max_cell_temp = input.core_data.max_cell_temp;
    }

    void handle_enqueue_acu_status_CAN_message();
    void handle_enqueue_acu_core_voltages_CAN_message();
    void handle_enqueue_acu_voltages_CAN_message();
    void handle_enqueue_acu_temps_CAN_message();

private:
    CCUCANInterfaceData_s _curr_data;
    ACUCoreData_s _acu_core_data;
    ACUAllData_s<ccu_interface_defaults::NUM_CELLS, ccu_interface_defaults::NUM_CELLTEMPS, ccu_interface_defaults::NUM_CHIPS> _acu_all_data;

    unsigned long _min_charging_enable_threshold;

    bool increment_and_loop_id(size_t id, size_t max_id, size_t increment = 1)
    {
        if (id == max_id - increment) {
            id = 0;
            return true;
        }
        else {
            id += increment;
            return false;
        }
    }
    CCUInterfaceParams_s _ccu_params;
};

using CCUInterfaceInstance = etl::singleton<CCUInterface>;

#endif // CCU_INTERFACE_H