#include "BMSFaultDataManager.h"
#include "etl/algorithm.h"

template <size_t num_chips>
void BMSFaultDataManager<num_chips>::update_from_valid_packets(
    const std::array<std::array<bool, ReadGroup_e::NUM_GROUPS>, num_chips>& valid_read_packets)
{
    size_t num_total_bms_packets = num_chips * ReadGroup_e::NUM_GROUPS;
    std::array<size_t, num_chips> chip_max_invalid_cmd_counts = {};
    std::array<size_t, ReadGroup_e::NUM_GROUPS> temp = {};
    size_t num_valid_packets = 0;
    
    for (size_t chip = 0; chip < valid_read_packets.size(); chip++)
    {
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_A] = (!valid_read_packets[chip][ReadGroup_e::CV_GROUP_A]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_A]+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_B] = (!valid_read_packets[chip][ReadGroup_e::CV_GROUP_B]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_B]+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_C] = (!valid_read_packets[chip][ReadGroup_e::CV_GROUP_C]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_C]+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_D] = (!valid_read_packets[chip][ReadGroup_e::CV_GROUP_D]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_D]+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_A] = (!valid_read_packets[chip][ReadGroup_e::AUX_GROUP_A]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_A]+1 : 0;
        _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_B] = (!valid_read_packets[chip][ReadGroup_e::AUX_GROUP_B]) ? _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_B]+1 : 0;
        num_valid_packets += static_cast<size_t>(valid_read_packets[chip][ReadGroup_e::CV_GROUP_A] + valid_read_packets[chip][ReadGroup_e::CV_GROUP_B] + valid_read_packets[chip][ReadGroup_e::CV_GROUP_C] + 
                              valid_read_packets[chip][ReadGroup_e::CV_GROUP_D] + valid_read_packets[chip][ReadGroup_e::AUX_GROUP_A] + valid_read_packets[chip][ReadGroup_e::AUX_GROUP_B]);

        temp = {_bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_A],
            _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_B],
            _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_C],
            _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::CV_GROUP_D],
            _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_A],
            _bms_fault_data.chip_invalid_cmd_counts[chip][ReadGroup_e::AUX_GROUP_B]};
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
