#include "BMSFaultDataManager.h"
#include "etl/algorithm.h"

template <size_t num_chips>
void BMSFaultDataManager<num_chips>::update_from_valid_packets(
    const std::array<ValidPacketData_s, num_chips>& valid_read_packets)
{
    size_t num_total_bms_packets = num_chips * sizeof(BMSFaultCountData_s);
    std::array<size_t, num_chips> chip_max_invalid_cmd_counts = {};
    std::array<size_t, sizeof(BMSFaultCountData_s)> temp = {};
    size_t num_valid_packets = 0;
    
    for (size_t chip = 0; chip < valid_read_packets.size(); chip++)
    {
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count = (!valid_read_packets[chip].valid_read_cells_1_to_3) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count = (!valid_read_packets[chip].valid_read_cells_4_to_6) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count = (!valid_read_packets[chip].valid_read_cells_7_to_9) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count = (!valid_read_packets[chip].valid_read_cells_10_to_12) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count = (!valid_read_packets[chip].valid_read_gpios_1_to_3) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count = (!valid_read_packets[chip].valid_read_gpios_4_to_6) ? _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count+1 : 0;
        num_valid_packets += static_cast<size_t>(valid_read_packets[chip].valid_read_cells_1_to_3 + valid_read_packets[chip].valid_read_cells_4_to_6 + valid_read_packets[chip].valid_read_cells_7_to_9 + 
                              valid_read_packets[chip].valid_read_cells_10_to_12 + valid_read_packets[chip].valid_read_gpios_1_to_3 + valid_read_packets[chip].valid_read_gpios_4_to_6);

        temp = {_bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count,
            _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count,
            _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count,
            _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count,
            _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count,
            _bms_fault_data.chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count};
        chip_max_invalid_cmd_counts[chip] = *etl::max_element(temp.begin(), temp.end());      
        _bms_fault_data.consecutive_invalid_packet_counts[chip] = chip_max_invalid_cmd_counts[chip];
    }
    _bms_fault_data.valid_packet_rate = static_cast<float>(static_cast<float>(num_valid_packets) / num_total_bms_packets);
    _bms_fault_data.max_consecutive_invalid_packet_count = *etl::max_element(chip_max_invalid_cmd_counts.begin(), chip_max_invalid_cmd_counts.end()); 
}

template <size_t num_chips>
const typename BMSFaultDataManager<num_chips>::BMSFaultData_s&
BMSFaultDataManager<num_chips>::get_fault_data() const
{
    return _bms_fault_data;
}
