#include <Arduino.h>

const uint8_t SENSOR_ROMS[6][8] = {
    { 0x28, 0xE0, 0xF4, 0x70, 0x11, 0x00, 0x00, 0xC7 },
    { 0x28, 0xC8, 0x92, 0x70, 0x11, 0x00, 0x00, 0x5D },
    { 0x28, 0xA6, 0xF5, 0x10, 0x11, 0x00, 0x00, 0x5D },
    { 0x28, 0xF9, 0x5A, 0x71, 0x11, 0x00, 0x00, 0x14 },
    { 0x28, 0x75, 0x42, 0x11, 0x11, 0x00, 0x00, 0x51 },
    { 0x28, 0x6B, 0xCE, 0x70, 0x11, 0x00, 0x00, 0xEC }
};

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600);
    delay(100);

    Serial2.end();
    Serial2.begin(4800);
    Serial2.write(0x00);
    delay(2);
    Serial2.end();
    Serial2.begin(9600);
    delay(2);
    while (Serial2.available()) Serial2.read();

    Serial2.write(0xC1);
    delay(2);
    while (Serial2.available()) Serial2.read();

    uint8_t detect_seq[5]  = {0x17, 0x45, 0x5B, 0x0F, 0x91};
    uint8_t detect_resp[5] = {0x16, 0x44, 0x5A, 0x00, 0x93};

    for (uint8_t i = 0; i < 5; i++)
    {
        Serial2.write(detect_seq[i]);
    }

    bool detect_ok = true;
    for (uint8_t i = 0; i < 5; i++)
    {
        uint32_t timeout = millis() + 20;
        while (!Serial2.available())
        {
            if (millis() > timeout) { detect_ok = false; break; }
        }
        uint8_t resp = Serial2.read();
        Serial.print("Detect["); Serial.print(i); Serial.print("]: got 0x");
        Serial.print(resp, HEX); Serial.print(" expected 0x");
        Serial.println(detect_resp[i], HEX);
        if (resp != detect_resp[i]) detect_ok = false;
    }

    Serial.println(detect_ok ? "DS2480B_Detect: success" : "DS2480B_Detect: FAILED");
}

void loop()
{
    Serial.println("--- START ---");

    // Set Strong Pullup Duration = 1048ms
    Serial2.write(0x3B);
    while (!Serial2.available()) {}
    Serial.print("SPUD: 0x"); Serial.println(Serial2.read(), HEX);

    for (uint8_t s = 0; s < 6; s++)
    {
        Serial.print("--- Sensor "); Serial.print(s); Serial.println(" ---");

        // === CONVERT WITH SPU ===

        // Reset
        Serial2.write(0xC1);
        while (!Serial2.available()) {}
        Serial.print("Convert Reset: 0x"); Serial.println(Serial2.read(), HEX);

        // Data mode
        Serial2.write(0xE1);

        // Match ROM
        while (Serial2.available()) Serial2.read();
        Serial2.write(0x55);
        while (!Serial2.available()) {}
        Serial2.read();

        for (uint8_t i = 0; i < 8; i++)
        {
            while (Serial2.available()) Serial2.read();
            Serial2.write(SENSOR_ROMS[s][i]);
            while (!Serial2.available()) {}
            Serial2.read();
        }

        // Arm SPU via check mode
        Serial2.write(0xE3);
        Serial2.write(0xEF);
        Serial2.write(0xF1);
        while (!Serial2.available()) {}
        Serial2.read();  // pulse response

        // Data mode
        Serial2.write(0xE1);

        // Convert T
        Serial2.write(0x44);
        while (!Serial2.available()) {}
        Serial2.read();  // echo

        // Wait for SPU done
        uint32_t timeout = millis() + 2000;
        while (!Serial2.available())
        {
            if (millis() > timeout)
            {
                Serial.println("SPU timeout");
                break;
            }
        }
        Serial.print("SPU: 0x"); Serial.println(Serial2.read(), HEX);

        // Disarm — flush with timeout
        Serial2.write(0xE3);
        Serial2.write(0xED);
        Serial2.write(0xF1);
        uint32_t disarm_timeout = millis() + 50;
        while (millis() < disarm_timeout)
        {
            if (Serial2.available()) Serial2.read();
        }

        // === READ SCRATCHPAD ===

        // Reset
        Serial2.write(0xC1);
        while (!Serial2.available()) {}
        Serial.print("Read Reset: 0x"); Serial.println(Serial2.read(), HEX);

        // Data mode
        Serial2.write(0xE1);

        // Match ROM
        while (Serial2.available()) Serial2.read();
        Serial2.write(0x55);
        while (!Serial2.available()) {}
        Serial2.read();

        for (uint8_t i = 0; i < 8; i++)
        {
            while (Serial2.available()) Serial2.read();
            Serial2.write(SENSOR_ROMS[s][i]);
            while (!Serial2.available()) {}
            Serial2.read();
        }

        // Read Scratchpad command
        while (Serial2.available()) Serial2.read();
        Serial2.write(0xBE);
        while (!Serial2.available()) {}
        Serial2.read();

        // Read 9 bytes
        uint8_t scratchpad[9];
        for (uint8_t i = 0; i < 9; i++)
        {
            Serial2.write(0xFF);
            while (!Serial2.available()) {}
            scratchpad[i] = Serial2.read();
        }

        // Switch to command mode before next sensor
        Serial2.write(0xE3);
        delay(15);
        while (Serial2.available()) Serial2.read();

        // CRC check
        uint8_t crc = 0;
        for (uint8_t i = 0; i < 9; i++)
        {
            uint8_t b = scratchpad[i];
            for (uint8_t j = 0; j < 8; j++)
            {
                uint8_t mix = (crc ^ b) & 0x01;
                crc >>= 1;
                if (mix) crc ^= 0x8C;
                b >>= 1;
            }
        }

        Serial.print("Scratchpad: ");
        for (uint8_t i = 0; i < 9; i++)
        {
            if (scratchpad[i] < 0x10) Serial.print('0');
            Serial.print(scratchpad[i], HEX);
            Serial.print(' ');
        }
        Serial.println();

        Serial.print("CRC: "); Serial.println(crc == 0 ? "PASS" : "FAIL");

        int16_t raw = (static_cast<int16_t>(scratchpad[1]) << 8) | scratchpad[0];
        float temp = raw / 16.0f;
        Serial.print("Temp: "); Serial.print(temp, 4); Serial.println(" C");
    }

    Serial.println("--- END ---");
    delay(2000);
}