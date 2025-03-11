#include "ACUEthernetInterface.h"
#include "SharedFirmwareTypes.h"

#include <algorithm>

hytech_msgs_VCRData_s ACUEthernetInterface::make_vcr_data_msg(const VCRData_s &shared_state)
{

}

void ACUEthernetInterface::receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData_s &msg_in, VCRData_s &shared_state)
{
    
}

void ACUEthernetInterface::receive_pb_msg_acu_core_data(const hytech_msgs_ACUCoreData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    
}

void ACUEthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.    
}

void ACUEthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    
}
	
void ACUEthernetInterface::copy_inverter_data(const InverterData_s &original, hytech_msgs_InverterData_s &destination)
{
    
}

void ACUEthernetInterface::handle_send_ethernet_data(const hytech_msgs_VCRData_s &data) {
    handle_ethernet_socket_send_pb<(size_t) 1024>(_drivebrain_ip, _acu_data_port, _udp_socket, data, hytech_msgs_VCRData_s_fields);
}