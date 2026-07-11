#ifndef ACUCANINTERFACEIMPL_H
#define ACUCANINTERFACEIMPL_H

/* Standard Library */
#include <cstdint>

/* ETL Library */
#include "etl/delegate.h"
#include "etl/singleton.h"

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "CANInterface.h"
#include "FlexCAN_T4.h"
#include "hytech.h"

/* Local Interface Includes */
#include "CCUInterface.h"
#include "EMInterface.h"

/* Globally accessible types */
constexpr size_t CAN_MSG_SIZE = sizeof(CAN_message_t);
using CANRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CAN_MSG_SIZE>;
using CANTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CAN_MSG_SIZE>;

template <CAN_DEV_TABLE CAN_DEV>
using FlexCAN_t = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;


/**
 * @brief This struct holds references to the interface objects that use decoded CAN messages. References only!
 */
struct CANInterfaces_s
{
    explicit CANInterfaces_s(CCUInterface &ccu_int, EMInterface &em_int) :
        ccu_interface(ccu_int),
        em_interface(em_int)
    {}

    CCUInterface &ccu_interface;
    EMInterface &em_interface;
};
using CANInterfacesInstance = etl::singleton<CANInterfaces_s>;

/**
 * @brief This struct holds the FlexCAN peripheral instances and their associated RX/TX ring buffers.
 */
struct ACUCANInterface_s
{
    explicit ACUCANInterface_s(etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> recv_switch_func) : can_recv_switch(recv_switch_func) {}

    FlexCAN_t<CAN2> CCU_CAN;
    CANRXBuffer_t ccu_can_rx_buffer;
    CANTXBuffer_t ccu_can_tx_buffer;

    FlexCAN_t<CAN3> EM_CAN;
    CANRXBuffer_t em_can_rx_buffer;

    etl::delegate<void (CANInterfaces_s &, const CAN_message_t &, uint32_t, CANInterfaceType_e)> can_recv_switch;
};
using ACUCANInterfaceInstance = etl::singleton<ACUCANInterface_s>;

namespace ACUCANInterfaceImpl
{
    void on_ccu_can_receive(const CAN_message_t &msg);

    void on_em_can_receive(const CAN_message_t &msg);

    /**
     * @brief Routes a decoded message to the appropriate interface based on CANID
     */
    void acu_recv_switch(CANInterfaces_s &interfaces, const CAN_message_t &msg, uint32_t millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBuffer_t &buffer, FlexCAN_T4_Base *can_interface);
};

#endif // ACUCANINTERFACEIMPL_H
