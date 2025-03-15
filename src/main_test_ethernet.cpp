#include <Arduino.h>
#include <cstdint>

#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"
// #include "hytech_msgs.pb.h"

#include <QNEthernet.h>

#include <array>
#include <cstring>


using namespace qindesign::network;
EthernetUDP send_socket; 
EthernetUDP recv_socket; 

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
        uint8_t buffer[8];
        size_t read_bytes = recv_socket.read(buffer, sizeof(buffer));
        recv_socket.read(buffer, 8);
        Serial.println("recvd data: ");
        for(size_t i =0; i<8; i++)
        {
            Serial.print(buffer[i]);
        }
        Serial.println();
    }

    send_socket.beginPacket(EthernetIPDefsInstance::instance().drivebrain_ip, EthernetIPDefsInstance::instance().DBData_port);
    uint8_t send_buf[8] = {0x45, 0x45, 0x45, 0x22, 0x22, 0x22, 0x22, 0x22};
    
    send_socket.write(send_buf, 8);
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
    delay(10);
    // Serial.println("loopin");
}