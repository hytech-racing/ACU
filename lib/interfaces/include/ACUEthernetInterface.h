#ifndef ACU_ETHERNET_INTERFACE_H
#define ACU_ETHERNET_INTERFACE_H

/* External Libraries */
#include "hytech_msgs.pb.h"
#include "QNEthernet.h"
#include "EthernetAddressDefs.h"

#include "SharedFirmwareTypes.h"

#include <etl/singleton.h>

using namespace qindesign::network;

namespace acu_ethernet_default_params
{
    constexpr const uint16_t ACU_CORE_DATA_PORT = EthernetIPDefsInstance::instance().ACUCoreData_port;
    constexpr const uint16_t ACU_ALL_DATA_PORT = EthernetIPDefsInstance::instance().ACUAllData_port
    constexpr const uint16_t VCR_DATA_PORT = EthernetIPDefsInstance::instance().VCRData_port;
    constexpr const uint16_t DB_DATA_PORT = EthernetIPDefsInstance::instance().DBData_port;
};


class ACUEthernetInterface 
{
public:
  ACUEthernetInterface()
  {
  };

  void init_ethernet_device();

private:
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
  EthernetUDP _send_socket;
  EthernetUDP _recv_socket;

  /* Send/Recv Ports */
  const uint16_t _send_port;
  const uint16_t _recv_port;

  /*  */
};

using ACUEthernetInterfaceInstance = etl::singleton<ACUEthernetInterface>;

#endif /* ACU_ETHERNET_INTERFACE_H */