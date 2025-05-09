#include "ACUEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "ht_can_version.h"
#include "hytech_msgs_version.h"
#include "Arduino.h"
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
    // no TCP Ethernet, just UDP
    handle_ethernet_socket_send_pb<hytech_msgs_ACUCoreData_size>(EthernetIPDefsInstance::instance().drivebrain_ip, 
                                                EthernetIPDefsInstance::instance().ACUCoreData_port,
                                                &_acu_core_data_send_socket, data, hytech_msgs_ACUCoreData_fields);
}

hytech_msgs_ACUCoreData ACUEthernetInterface::make_acu_core_data_msg(const ACUCoreData_s &shared_state)
{
    hytech_msgs_ACUCoreData out;
    
    out.pack_voltage = shared_state.pack_voltage;
    out.min_cell_voltage = shared_state.min_cell_voltage;
    out.max_cell_voltage = shared_state.max_cell_voltage;
    out.avg_cell_voltage = shared_state.avg_cell_voltage;
    out.max_cell_temp = shared_state.max_cell_temp;

    out.measured_glv = shared_state.measured_glv;
    out.max_board_temp = shared_state.max_board_temp;
    out.measured_pack_voltage = shared_state.measured_pack_out_voltage;
    out.measured_tractive_system_voltage = shared_state.measured_ts_out_voltage;

    return out;
}

hytech_msgs_ACUAllData ACUEthernetInterface::make_acu_all_data_msg(const ACUAllDataType_s &shared_state)
{
    hytech_msgs_ACUAllData out = {};
    out.has_core_data = true;
    out.core_data = make_acu_core_data_msg(shared_state.core_data);

    out.cell_voltages_count = _num_cells;
    std::copy(shared_state.cell_voltages.data(), shared_state.cell_voltages.data() + _num_cells, out.cell_voltages);
    
    out.cell_temperatures_count = _num_celltemps;
    std::copy(shared_state.cell_temps.data(), shared_state.cell_temps.data() + _num_celltemps, out.cell_temperatures);
    
    out.invalid_packet_chip_counts_count = _num_chips;
    std::copy(shared_state.consecutive_invalid_packet_counts.data(), shared_state.consecutive_invalid_packet_counts.data() + _num_chips, out.invalid_packet_chip_counts);

    out.board_temperatures_count = _num_chips;
    std::copy(shared_state.board_temps.data(), shared_state.board_temps.data() + _num_chips, out.board_temperatures);

    out.max_consecutive_invalid_packet_count = shared_state.max_consecutive_invalid_packet_count;
    out.max_cell_voltage_id = shared_state.max_cell_voltage_id;
    out.min_cell_voltage_id = shared_state.min_cell_voltage_id;
    out.max_cell_temp_id = shared_state.max_cell_temp_id;
    out.measured_bspd_current = shared_state.measured_bspd_current;
    out.valid_packet_rate = shared_state.valid_packet_rate;
    out.SoC = shared_state.SoC;
    out.SoH = -1;
    /* Firmware Version Hash Assignment */
    out.has_firmware_version_info = true;
    out.firmware_version_info.project_is_dirty = shared_state.fw_version_info.project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = shared_state.fw_version_info.project_on_main_or_master;
    std::copy(shared_state.fw_version_info.fw_version_hash.begin(), shared_state.fw_version_info.fw_version_hash.end(), out.firmware_version_info.git_hash);
    out.has_msg_versions = true;
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;
    std::copy(version, version + std::min(strlen(version), sizeof(out.msg_versions.ht_proto_version) - 1), out.msg_versions.ht_proto_version);    
    out.msg_versions.ht_proto_version[sizeof(out.msg_versions.ht_proto_version) - 1] = '\0';
    return out;
}