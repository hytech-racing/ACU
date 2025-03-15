#ifndef ACU_ETHERNET_INTERFACE_H
#define ACU_ETHERNET_INTERFACE_H

/* External Libraries */
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "SharedFirmwareTypes.h"

namespace ACUEthernetInterface
{
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

    /**
     * Helper function to copy veh_vec data.
     *
     * @param original A populated instance of a veh_vec.
     * @param destination A reference to an unpopulated instance of veh_vec.
     * @post The destination veh_vec will be populated with the data from the original.
     */
    // template <typename from_T, typename to_T>
    // void copy_veh_vec_members(const from_T& from, to_T& to)
    // {
    //   to.FL = from.FL;
    //   to.FR = from.FR;
    //   to.RL = from.RL;
    //   to.RR = from.RR;
    // }

    //void handle_send_ethernet_data(const hytech_msgs_VCRData_s &data);
};

#endif /* ACU_ETHERNET_INTERFACE_H */