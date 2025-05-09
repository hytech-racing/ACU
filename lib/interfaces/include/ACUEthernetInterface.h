#ifndef ACU_ETHERNET_INTERFACE_H
#define ACU_ETHERNET_INTERFACE_H

/* External Libraries */
#include "hytech_msgs.pb.h"
#include "QNEthernet.h"
#include "EthernetAddressDefs.h"
#include "ProtobufMsgInterface.h"

#include "SharedFirmwareTypes.h"
#include "shared_types.h"

#include "device_fw_version.h"

#include <etl/singleton.h>

using namespace qindesign::network;

namespace acu_ethernet_params {
  constexpr const uint8_t num_cells = 126;
  constexpr const uint8_t num_celltemps = 48;
  constexpr const uint8_t num_chips = 12;
};

class ACUEthernetInterface
{
public:
  ACUEthernetInterface(uint8_t num_cells = acu_ethernet_params::num_cells, 
                        uint8_t num_celltemps = acu_ethernet_params::num_celltemps,
                        uint8_t num_chips = acu_ethernet_params::num_chips) :
                        _num_cells(num_cells),
                        _num_celltemps(num_celltemps),
                        _num_chips(num_chips) {};

  void init_ethernet_device();

  void handle_send_ethernet_acu_all_data(const hytech_msgs_ACUAllData &data);

  void handle_send_ethernet_acu_core_data(const hytech_msgs_ACUCoreData &data);

  /**
   * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_ACUCoreData_s.
   *
   * @param shared_state Minimum data ACU must send for car to run.
   * @return A populated instance of the outgoing protoc struct.
   */
  hytech_msgs_ACUCoreData make_acu_core_data_msg(const ACUCoreData_s &shared_state);

  /**
   * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_ACUAllData_s.
   *
   * @param shared_state Detailed, unprocessed data from ACU sensors.
   * @return A populated instance of the outgoing protoc struct.
   */
  hytech_msgs_ACUAllData make_acu_all_data_msg(const ACUAllDataType_s &shared_state);

private:
  /* Ethernet Sockets */
  EthernetUDP _acu_core_data_send_socket;
  EthernetUDP _acu_all_data_send_socket;
  EthernetUDP _vcr_data_recv_socket;
  EthernetUDP _db_data_recv_socket;

  const uint8_t _num_cells = 0;
  const uint8_t _num_celltemps = 0;
  const uint8_t _num_chips = 0;
};

using ACUEthernetInterfaceInstance = etl::singleton<ACUEthernetInterface>;

#endif /* ACU_ETHERNET_INTERFACE_H */