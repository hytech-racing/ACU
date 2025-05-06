#include <Arduino.h>

#include "CANInterface.h"

const FlexCAN_T4<CAN3> MAIN_CAN;
const size_t CAN_BAUDRATE = 500000;
const size_t delay = 10;
const uint8_t test_msg_id = 0x11;
const uint8_t test_msg_len = 1;
const uint8_t test_msg_buf0 = 0x45;
const uint8_t buf_len = 8;

void on_recv(const CAN_message_t &msg)
{
    Serial.println("msg recvd");
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (auto b : msg.buf) {
      Serial.print(b); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
}
    

void setup()
{
    
    handle_CAN_setup(MAIN_CAN, CAN_BAUDRATE, &on_recv);
}

void loop()
{
    delay(delay);
    CAN_message_t test_msg;
    test_msg.id = test_msg_id;
    test_msg.len = test_msg_len;
    test_msg.buf[0] = test_msg_buf0;
    MAIN_CAN.write(test_msg);
}