#ifndef BMSFaultDataManager_H
#define BMSFaultDataManager_H

#include <etl/singleton.h>

#include "BMSDriverGroup.h"  // for ValidPacketData_s

template <size_t num_chips>
class BMSFaultDataManager
{
public:
    struct BMSFaultCountData_s {
        uint8_t invalid_cell_1_to_3_count  = 0;
        uint8_t invalid_cell_4_to_6_count  = 0;
        uint8_t invalid_cell_7_to_9_count  = 0;
        uint8_t invalid_cell_10_to_12_count= 0;
        uint8_t invalid_gpio_1_to_3_count  = 0;
        uint8_t invalid_gpio_4_to_6_count  = 0;
    };

    struct BMSFaultData_s {
        std::array<size_t, num_chips> consecutive_invalid_packet_counts{};
        float  valid_packet_rate = 0.0f;                              
        size_t max_consecutive_invalid_packet_count = 0;                   
        std::array<BMSFaultCountData_s, num_chips> chip_invalid_cmd_counts{};
    };

    void update_from_valid_packets( const std::array<ValidPacketData_s, num_chips>& valid_read_packets);

    const BMSFaultData_s& get_fault_data() const;

private:
    BMSFaultData_s _bms_fault_data{};
};

template <size_t num_chips>
using BMSFaultDataManagerInstance = etl::singleton<BMSFaultDataManager<num_chips>>;

#include "BMSFaultDataManager.tpp"

#endif 
