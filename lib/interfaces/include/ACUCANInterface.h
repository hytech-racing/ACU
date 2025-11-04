#ifndef ACUCANINTERFACE_H
#define ACUCANINTERFACE_H

#include <cstdint>
#include <tuple>
#include <utility>

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "FlexCAN_T4.h"
#include "CANInterface.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include "hytech.h" // generated CAN library

#include "ACUCANBuffers.h"
#include "ACUController.h"
#include "VCRInterface.h"
#include "EMInterface.h"
#include "CCUInterface.h"

/* FlexCAN type alias */
template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_t = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

/* CANInterfaces struct - holds references to all CAN-connected interfaces */
template <size_t num_cells, size_t num_celltemps, size_t num_chips>
struct CANInterfaces_s {
    explicit CANInterfaces_s(CCUInterface<num_cells, num_celltemps, num_chips> &ccu_int, EMInterface &em_int) :
        ccu_interface(ccu_int),
        em_interface(em_int) {}
    
    CCUInterface<num_cells, num_celltemps, num_chips> & ccu_interface;
    EMInterface & em_interface;
};

template <size_t num_cells, size_t num_celltemps, size_t num_chips>
using CANInterfacesInstance = etl::singleton<CANInterfaces_s<num_cells, num_celltemps, num_chips>>;

/* ACUCANInterface namespace - Main CAN interface handler functions */
namespace ACUCANInterface {
    // CAN hardware instances
    extern FlexCAN_t<CAN3> CCU_CAN;
    extern FlexCAN_t<CAN2> EM_CAN;

    // CAN receive callbacks
    void on_ccu_can_receive(const CAN_message_t &msg);
    void on_em_can_receive(const CAN_message_t &msg);

    // Main CAN message handler (templated free function)
    template <size_t num_cells, size_t num_celltemps, size_t num_chips>
    void acu_CAN_recv(CANInterfaces_s<num_cells, num_celltemps, num_chips> &interfaces,
                      const CAN_message_t &msg,
                      unsigned long millis);

    // Send all queued CAN messages
    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
}

#include "ACUCANInterface.tpp"

#endif
