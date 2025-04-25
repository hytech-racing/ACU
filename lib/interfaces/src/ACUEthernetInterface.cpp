#include "ACUEthernetInterface.h"
#include "SharedFirmwareTypes.h"

#include <algorithm>

void ACUEthernetInterface::init_ethernet_device() {
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().acu_ip, EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    _acu_core_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    _acu_all_data_send_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);
    _vcr_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    _db_data_recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);
}

void ACUEthernetInterface::handle_send_ethernet_acu_all_data(const hytech_msgs_ACUAllData &data) {

    handle_ethernet_socket_send_pb<hytech_msgs_ACUAllData_size>(EthernetIPDefsInstance::instance().drivebrain_ip, 
                                                EthernetIPDefsInstance::instance().ACUAllData_port,
                                                &_acu_all_data_send_socket, data, hytech_msgs_ACUAllData_fields);
}

void ACUEthernetInterface::handle_send_ethernet_acu_core_data(const hytech_msgs_ACUCoreData &data) {
    handle_ethernet_socket_send_pb<hytech_msgs_ACUCoreData_size>(EthernetIPDefsInstance::instance().drivebrain_ip, 
                                                EthernetIPDefsInstance::instance().ACUCoreData_port,
                                                &_acu_core_data_send_socket, data, hytech_msgs_ACUCoreData_fields);


    // handle_ethernet_socket_send_pb<hytech_msgs_ACUCoreData_size>(EthernetIPDefsInstance::instance().acu_ip, 
    //                                             EthernetIPDefsInstance::instance().DBData_port,
    //                                             &_acu_core_data_send_socket, data, hytech_msgs_ACUCoreData_fields);
}

hytech_msgs_ACUCoreData ACUEthernetInterface::make_acu_core_data_msg(const ACUCoreData_s &shared_state)
{
    hytech_msgs_ACUCoreData out;
    
    out.pack_voltage = shared_state.pack_voltage;
    out.min_cell_voltage = shared_state.min_cell_voltage;
    out.max_cell_voltage = shared_state.max_cell_voltage;
    out.avg_cell_voltage = shared_state.avg_cell_voltage;
    out.max_cell_temp = shared_state.max_cell_temp;

    return out;
}

hytech_msgs_ACUAllData ACUEthernetInterface::make_acu_all_data_msg(const ACUAllDataType_s &shared_state)
{
    hytech_msgs_ACUAllData out = {};
    out.cell_voltages_count = 126;
    for (size_t i = 0; i < 126; ++i)
    {
        out.cell_voltages[i] = shared_state.cell_voltages[i];
    }
    out.cell_temperatures_count = 48;
    for (size_t i = 0; i < 48; i++)
    {
        out.cell_temperatures[i] = shared_state.cell_temps[i];
    }
    out.invalid_packet_chip_counts_count = 12;
    for (size_t i = 0; i < 12; i++) {
        out.invalid_packet_chip_counts[i] = shared_state.consecutive_invalid_packet_counts[i];
    }

    out.has_core_data = true;
    out.core_data = make_acu_core_data_msg(shared_state.core_data);
    out.max_consecutive_invalid_packet_count = shared_state.max_consecutive_invalid_packet_count;
    out.max_cell_voltage_id = shared_state.max_cell_voltage_id;
    out.min_cell_voltage_id = shared_state.min_cell_voltage_id;
    out.max_cell_temp_id = shared_state.max_cell_temp_id;
    out.measured_tractive_system_voltage = shared_state.measured_tractive_system_voltage;
    out.measured_pack_voltage = shared_state.measured_pack_voltage;
    out.measured_bspd_current = shared_state.measured_bspd_current;

    /* Firmware Version Hash Assignment */
    out.firmware_version_info.project_is_dirty = shared_state.fw_version_info.project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = shared_state.fw_version_info.project_on_main_or_master;
    std::copy(shared_state.fw_version_info.fw_version_hash.begin(), shared_state.fw_version_info.fw_version_hash.end(), out.firmware_version_info.git_hash);
    
    return out;
}

void ACUEthernetInterface::receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, ACUCoreData_s &shared_state)
{
    shared_state.avg_cell_voltage = msg_in.ams_data.average_cell_voltage;
    shared_state.max_cell_temp = msg_in.ams_data.max_temp_celsius;
    shared_state.min_cell_voltage = msg_in.ams_data.min_cell_voltage;
    shared_state.pack_voltage = msg_in.ams_data.total_pack_voltage;
}