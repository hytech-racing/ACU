#ifndef BMSFaultDataManager_H
#define BMSFaultDataManager_H

#include <etl/singleton.h>

#include "BMSDriverGroup.h"  // for ValidPacketData_s

template <size_t num_chips>
class BMSFaultDataManager
{
public:
    struct BMSFaultData_s {
        std::array<size_t, num_chips> consecutive_invalid_packet_counts{};
        float  valid_packet_rate = 0.0f;                              
        size_t max_consecutive_invalid_packet_count = 0;                   
        std::array<std::array<bool, ReadGroup_e::NUM_GROUPS>, num_chips> chip_invalid_cmd_counts{};
    };

    void update_from_valid_packets( const std::array<std::array<bool, ReadGroup_e::NUM_GROUPS>, num_chips>& valid_read_packets);

    const BMSFaultData_s& get_fault_data() const;

private:
    BMSFaultData_s _bms_fault_data{};
};

template <size_t num_chips>
using BMSFaultDataManagerInstance = etl::singleton<BMSFaultDataManager<num_chips>>;

#include "BMSFaultDataManager.tpp"

#endif 
