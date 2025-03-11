#ifndef ACU_ETHERNET_INTERFACE_H
#define ACU_ETHERNET_INTERFACE_H

/* ACU Local Dependencies*/
#include "ACU_Constants.h"
#include "ACU_Globals.h"

/* External Libraries */
#include "hytech_msgs.pb.h"
#include "ProtobufMsgInterface.h"
#include "SharedFirmwareTypes.h"

class ACUEthernetInterface
{
private:
    IPAddress _drivebrain_ip; // Private member variable for drivebrain IP
    uint16_t _acu_data_port;         // Private member variable for ACU data port
    qindesign::network::EthernetUDP * _udp_socket;            // Private member variable for UDP socket

public:
    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCRData_s.
     *
     * @param shared_state The current VCR state, which includes both interface and system data.
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCRData_s make_vcr_data_msg(const VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from ACU and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUCoreData.
     */
    void receive_pb_msg_acu_core_data(const hytech_msgs_ACUCoreData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Function to take a populated protoc struct from ACU and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData_s &msg_in, VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from the drivebrain and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Function to take a populated protoc struct from VCF and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Helper function to copy an instance of InverterData_s to the protoc struct hytech_msgs_InverterData_s.
     * @param original A populated instance of the InverterData_s defined in shared firmware types.
     * @param destination The destination protoc struct.
     * @post The destination struct will be populated with the data from original.
     */
    void copy_inverter_data(const InverterData_s &original, hytech_msgs_InverterData_s &destination);

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

    void handle_send_ethernet_data(const hytech_msgs_VCRData_s &data);
};

#endif /* ACU_ETHERNET_INTERFACE_H */