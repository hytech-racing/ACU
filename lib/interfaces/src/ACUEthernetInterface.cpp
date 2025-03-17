#include "ACUEthernetInterface.h"
#include "SharedFirmwareTypes.h"

#include <algorithm>

void ACUEthernetInterface::init_ethernet_device() {
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().acu_ip,  EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    _acu_core_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    _acu_all_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);
    _vcr_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    _db_data_recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);
}

void ACUEthernetInterface::handle_send_ethernet_acu_all_data(const hytech_msgs_ACUAllData_s &data) {
    handle_ethernet_socket_send_pb<(size_t)1024>(EthernetIPDefsInstance::instance().acu_ip, 
                                                EthernetIPDefsInstance::instance().DBData_port,
                                                &_acu_all_data_send_socket, data, hytech_msgs_ACUAllData_s_fields);
}

void ACUEthernetInterface::handle_send_ethernet_acu_core_data(const hytech_msgs_ACUCoreData_s &data) {
    handle_ethernet_socket_send_pb<(size_t)1024>(EthernetIPDefsInstance::instance().acu_ip, 
                                                EthernetIPDefsInstance::instance().VCRData_port,
                                                &_acu_core_data_send_socket, data, hytech_msgs_ACUCoreData_s_fields);

    handle_ethernet_socket_send_pb<(size_t)1024>(EthernetIPDefsInstance::instance().acu_ip, 
                                                EthernetIPDefsInstance::instance().DBData_port,
                                                &_acu_core_data_send_socket, data, hytech_msgs_ACUCoreData_s_fields);
}

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

    for (size_t i = 0; i < out.voltages_count; ++i)
    {
        out.voltages[i] = shared_state.voltages[i];
    }

    for (size_t i = 0; i < out.cell_temperatures_count; ++i)
    {
        out.cell_temperatures[i] = shared_state.cell_temperatures[i];
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