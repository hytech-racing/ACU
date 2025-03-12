#include "ACUEthernetInterface.h"
#include "SharedFirmwareTypes.h"

#include <algorithm>

hytech_msgs_ACUCoreData_s ACUEthernetInterface::make_acu_core_data_msg(const ACUCoreData_s &shared_state)
{
    hytech_msgs_ACUCoreData_s out;
    
    out.pack_voltage = shared_state.pack_voltage;
    out.min_cell_voltage = shared_state.min_cell_voltage;
    out.avg_cell_voltage = shared_state.avg_cell_voltage;
    out.max_cell_temp = shared_state.max_cell_temp;

    return out;
}

hytech_msgs_ACUAllData_s ACUEthernetInterface::make_acu_all_data_msg(const ACUAllData_s &shared_state)
{
    hytech_msgs_ACUAllData_s out;

    for (uint32_t i = 0; i < out.voltages_count; ++i)
    {
        out.voltages[i] = shared_state.voltages[i];
    }

    for (uint32_t i = 0; i < out.cell_temperatures_count; ++i)
    {
        out.cell_temperatures[i] = shared_state.cell_temperatures[i];
    }

    for (uint32_t i = 0; i < out.board_humidities_count; ++i)
    {
        out.board_humidities[i] = shared_state.board_humidities[i];
    }

    return out;
}

void ACUEthernetInterface::receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, ACUCoreData_s &shared_state)
{
    shared_state.avg_cell_voltage = msg_in.ams_data.average_cell_voltage;
    shared_state.max_cell_temp = msg_in.ams_data.max_temp_celsius;
    shared_state.min_cell_voltage = msg_in.ams_data.min_cell_voltage;
    shared_state.pack_voltage = msg_in.ams_data.total_pack_voltage;
}