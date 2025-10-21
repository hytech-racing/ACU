#ifndef SHAREDTYPES_H
#define SHAREDTYPES_H

template<size_t num_cells, size_t num_celltemps, size_t num_boardtemps>
struct ACUData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    volt avg_cell_voltage;
    std::array<volt, num_cells> voltages;
    celsius max_cell_temp; 
    celsius min_cell_temp; 
    celsius max_board_temp;
    float SoC;
    std::array<bool, num_cells> cell_balancing_statuses;
    std::array<celsius, num_celltemps> cell_temps;
    std::array<celsius, num_boardtemps> board_temps;
    size_t max_consecutive_invalid_packet_count;
    bool charging_enabled;
    uint32_t last_bms_not_ok_eval;
    bool veh_imd_fault_latched;
    bool veh_bms_fault_latched;
    bool veh_shdn_out_latched; // requires shdn_out voltage to be above 10 V for 10 ms
    uint32_t last_valid_shdn_out_ms; // last time when shdn_out was > 10V
    bool bms_ok; // False when one of the three shutdown conditions is met
};

struct BMSFaultCountData_s {
    uint8_t invalid_cell_1_to_3_count;
    uint8_t invalid_cell_4_to_6_count;
    uint8_t invalid_cell_7_to_9_count;
    uint8_t invalid_cell_10_to_12_count;
    uint8_t invalid_gpio_1_to_3_count;
    uint8_t invalid_gpio_4_to_6_count;
};

template<size_t num_chips>
struct ACUFaultData_s {
    std::array<size_t, num_chips> consecutive_invalid_packet_counts;
    size_t max_consecutive_invalid_packet_count;
    std::array<BMSFaultCountData_s, num_chips> chip_invalid_cmd_counts;
};

#endif
