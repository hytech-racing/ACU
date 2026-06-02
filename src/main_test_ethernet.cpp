#include <Arduino.h>
#include <cstdint>

#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

#include <QNEthernet.h>

#include <array>
#include <cstring>

using namespace qindesign::network;
// const EthernetUDP send_socket; 
// const EthernetUDP recv_socket; 

// const uint32_t delay_ms = 10;
// const size_t buf_len = 8;

// void init_ethernet_device()
// {
//     Ethernet.begin(EthernetIPDefsInstance::instance().acu_ip,  EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
//     send_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
//     recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);
// }

// void test_ethernet()
// {
//     int packet_size = recv_socket.parsePacket();
//     if (packet_size > 0)
//         {
//         std::array<uint8_t, buf_len> buffer;
//         size_t read_bytes = recv_socket.read(buffer.data(), buffer.size());
//         recv_socket.read(buffer.data(), buf_len);
//         Serial.println("recvd data: ");
//         for(uint8_t i =0; i<buf_len; i++)
//         {
//             Serial.print(buffer[i]);
//         }
//         Serial.println();
//     }

//     send_socket.beginPacket(EthernetIPDefsInstance::instance().drivebrain_ip, EthernetIPDefsInstance::instance().DBData_port);
//     std::array<uint8_t, buf_len> send_buf = {0x45, 0x45, 0x45, 0x22, 0x22, 0x22, 0x22, 0x22};
//     send_socket.write(send_buf.data(), send_buf.size());
//     send_socket.endPacket();
// }

// void setup()
// {
//     EthernetIPDefsInstance::create();
//     init_ethernet_device();
// }

// void loop()
// {
//     test_ethernet();
//     delay(delay_ms);
//     // Serial.println("loopin");
// }

// Static setup
static const IPAddress MY_IP     (192, 168, 1,  11);
static const IPAddress PEER_IP   (192, 168, 1,  10);
static const IPAddress SUBNET    (255, 255, 255,  0);
static const IPAddress GATEWAY   (192, 168, 1,   1);
static constexpr uint16_t PORT   = 5005;
static constexpr uint32_t SEND_INTERVAL_MS = 500;

// Globals
EthernetUDP udp;
uint32_t    lastSend  = 0;
uint32_t    sendCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Ethernet.begin(MY_IP, SUBNET, GATEWAY);

  // Wait for link
  Serial.print("Waiting for link...");
  while (!Ethernet.linkState()) {
    Serial.print('.');
    delay(250);
  }
  Serial.println(" linked!");
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  // Bind UDP socket
  if (!udp.begin(PORT)) {
    Serial.println("ERROR: failed to bind UDP socket!");
    while (true) {}
  }
  Serial.print("Listening on UDP port ");
  Serial.println(PORT);
  Serial.println("Ready.\n");
}

void loop() {
  // Receive
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char buf[256];
    int  len = udp.read(buf, sizeof(buf) - 1);
    if (len < 0) len = 0;
    buf[len] = '\0';

    Serial.print("[RX] from ");
    Serial.print(udp.remoteIP());
    Serial.print("  \"");
    Serial.print(buf);
    Serial.println("\"");
  }

  // Send every SEND_INTERVAL_MS
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendCount++;

    char msg[64];
    snprintf(msg, sizeof(msg), "Hello from ACU, packet %lu", sendCount);

    udp.beginPacket(PEER_IP, PORT);
    udp.write((const uint8_t*)msg, strlen(msg));
    bool ok = udp.endPacket();

    Serial.print("[TX] -> ");
    Serial.print(PEER_IP);
    Serial.print("  \"");
    Serial.print(msg);
    Serial.print("\"  ");
    Serial.println(ok ? "OK" : "FAILED");
  }
}