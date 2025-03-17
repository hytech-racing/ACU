#ifndef ACU_ETHERNET_INTERFACE_H
#define ACU_ETHERNET_INTERFACE_H

/* External Libraries */
#include "hytech_msgs.pb.h"
#include "QNEthernet.h"
#include "EthernetAddressDefs.h"
#include "ProtobufMsgInterface.h"

#include "SharedFirmwareTypes.h"

#include <etl/singleton.h>

using namespace qindesign::network;

class ACUEthernetInterface 
{
public:
  ACUEthernetInterface() {};

  void init_ethernet_device();

  void handle_send_ethernet_acu_all_data(const hytech_msgs_ACUAllData_s &data);

  void handle_send_ethernet_acu_core_data(const hytech_msgs_ACUCoreData_s &data);

  /**
   * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_ACUCoreData_s.
   *
   * @param shared_state Minimum data ACU must send for car to run.
   * @return A populated instance of the outgoing protoc struct.
   */
  hytech_msgs_ACUCoreData_s make_acu_core_data_msg(const ACUCoreData_s &shared_state);

  /**
   * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_ACUAllData_s.
   *
   * @param shared_state Detailed, unprocessed data from ACU sensors.
   * @return A populated instance of the outgoing protoc struct.
   */
  hytech_msgs_ACUAllData_s make_acu_all_data_msg(const ACUAllData_s &shared_state);
  /**
   * Function to take a populated protoc struct from VCR and update ACUCoreData.
   *
   * @param msg_in A reference to a populated protoc struct.
   * @param shared_state A reference to ACUCoreData_s.
   *
   * @post After this function completes, shared_state will have updated contents of VCRData.
   */
  void receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, ACUCoreData_s &shared_state);

private: 
  /* Ethernet Sockets */
  EthernetUDP _acu_core_data_send_socket;
  EthernetUDP _acu_all_data_send_socket;
  EthernetUDP _vcr_data_recv_socket;
  EthernetUDP _db_data_recv_socket;
  
  /* IP Address */

};

using ACUEthernetInterfaceInstance = etl::singleton<ACUEthernetInterface>;

#endif /* ACU_ETHERNET_INTERFACE_H */