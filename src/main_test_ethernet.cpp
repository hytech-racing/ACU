#include <Arduino.h>
#include <cstdint>

#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

#include <QNEthernet.h>

#include <array>
#include <cstring>

using namespace qindesign::network;
const EthernetUDP send_socket; 
const EthernetUDP recv_socket; 

const uint32_t delay_ms = 10;
const size_t buf_len = 8;

void init_ethernet_device()
{
    Ethernet.begin(EthernetIPDefsInstance::instance().acu_ip,  EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    send_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);
}

void test_ethernet()
{
    int packet_size = recv_socket.parsePacket();
    if (packet_size > 0)
        {
        std::array<uint8_t, buf_len> buffer;
        size_t read_bytes = recv_socket.read(buffer.data(), buffer.size());
        recv_socket.read(buffer.data(), buf_len);
        Serial.println("recvd data: ");
        for(uint8_t i =0; i<buf_len; i++)
        {
            Serial.print(buffer[i]);
        }
        Serial.println();
    }

    send_socket.beginPacket(EthernetIPDefsInstance::instance().drivebrain_ip, EthernetIPDefsInstance::instance().DBData_port);
    std::array<uint8_t, buf_len> send_buf = {0x45, 0x45, 0x45, 0x22, 0x22, 0x22, 0x22, 0x22};
    send_socket.write(send_buf.data(), send_buf.size());
    send_socket.endPacket();
}

void setup()
{
    EthernetIPDefsInstance::create();
    init_ethernet_device();
}

void loop()
{
    test_ethernet();
    delay(delay_ms);
    // Serial.println("loopin");
}