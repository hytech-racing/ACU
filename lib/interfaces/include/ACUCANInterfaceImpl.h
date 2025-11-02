#ifndef ACUCANINTERFACEIMPL_H
#define ACUCANINTERFACEIMPL_H

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

#include "ACUController.h"
#include "CCUInterface.h"
#include "VCRInterface.h"
#include "EMInterface.h"

const size_t CAN_MSG_SIZE = sizeof(CAN_message_t);
using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CAN_MSG_SIZE>;
using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CAN_MSG_SIZE>;

/* RX buffers for CAN extern declarations*/
template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_t = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

struct CANInterfaces_s {
    explicit CANInterfaces_s(CCUInterface &ccu_int, EMInterface &em_int) :
        ccu_interface(ccu_int),
        em_interface(em_int) {}
    
    CCUInterface & ccu_interface;
    EMInterface & em_interface;
};

using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;
namespace ACUCANInterfaceImpl {
    extern CANRXBuffer_t ccu_can_rx_buffer;
    extern CANRXBuffer_t em_can_rx_buffer;
    extern CANTXBuffer_t ccu_can_tx_buffer;

    extern FlexCAN_t<CAN3> CCU_CAN;
    extern FlexCAN_t<CAN2> EM_CAN;

    void on_ccu_can_receive(const CAN_message_t &msg);
    void on_em_can_receive(const CAN_message_t &msg);
    
    void acu_CAN_recv(CANInterfaces_s &interfaces, const CAN_message_t &msg, unsigned long millis);

    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
    
}; // namespace ACUCANInterfaceImpl

#endif // ACUCANINTERFACEIMPL_H
