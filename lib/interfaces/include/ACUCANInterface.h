#ifndef __ACU_CAN_INTERFACE_H__
#define __ACU_CAN_INTERFACE_H__

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

using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/
template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

struct CANInterfaces {
    explicit CANInterfaces(CCUInterface &ccu_int, EMInterface &em_int) :
        ccu_interface(ccu_int),
        em_interface(em_int) {}
    
    CCUInterface & ccu_interface;
    EMInterface & em_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces>;

extern FlexCAN_T4<CAN3> CCU_CAN;
extern FlexCAN_T4<CAN2> EM_CAN;

namespace ACUCANInterfaceImpl {

    extern CANRXBufferType ccu_can_rx_buffer;
    extern CANRXBufferType em_can_rx_buffer;
    extern CANTXBufferType ccu_can_tx_buffer;

    void on_ccu_can_receive(const CAN_message_t &msg);
    void on_em_can_receive(const CAN_message_t &msg);
    
    void acu_CAN_recv(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis);

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);
    
}; // namespace ACUCANInterfaceImpl

#endif // __ACU_CAN_INTERFACE_H__
