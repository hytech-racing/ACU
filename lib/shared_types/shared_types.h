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
    celsius max_board_temp;
    float SoC;

    std::array<bool, num_cells> cell_balancing_statuses;
    std::array<celsius, num_celltemps> cell_temps;
    std::array<celsius, num_boardtemps> board_temps;

    size_t max_consecutive_invalid_packet_count;

    bool charging_enabled;
    bool acu_ok; // False when one of the three shutdown conditions is met (see AMSSystem header)
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
