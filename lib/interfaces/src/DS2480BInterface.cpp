#include "DS2480BInterface.h"

void DS2480BInterface::init()
{
    Serial2.begin(_ds2480b_params.baud_rate);
    delayMicroseconds(10);
    OWDetect();
}

bool DS2480BInterface::OWDetect()
{
    // App Note: "If break is not available on the host UART then
    // switching to a slower baud rate and sending a zero byte can simulate a break"
    // Send break via null byte at 4800 bau
    Serial2.end();
    Serial2.begin(_ds2480b_params.baud_rate / 2);
    Serial2.write(_ds2480b_params.commands.break_cmd);
    delay(2);

    Serial2.end();
    Serial2.begin(_ds2480b_params.baud_rate);
    delay(2);
    _flushRXBuffer();

    Serial2.write(_ds2480b_params.commands.timing_byte);
    delay(2);
    _flushRXBuffer();

    _curr_mode = DS2480B_Mode::COMMAND_MODE;

    // Send 5-byte flex config packet
    for (uint8_t detect_cmd_sent : _ds2480b_params.commands.detect_sequence)
    {
        Serial2.write(detect_cmd_sent);
    }

    // Read and validate 5-byte response
    for (uint8_t expected_resp : _ds2480b_params.commands.detect_response)
    {
        uint32_t timeout = millis() + 10;
        while (!Serial2.available())
        {
            if (millis() > timeout)
            {
                Serial.println("OWDetect: timeout waiting for response");
                return false;
            }
        }

        uint8_t response = static_cast<uint8_t>(Serial2.read());

        if (response != expected_resp)
        {
            Serial.print("DS2480B_Detect: bad byte ");
            Serial.print(" got 0x");
            Serial.print(response, HEX);
            Serial.print(" expected 0x");
            Serial.println(expected_resp, HEX);
            return false;
        }
    }

    Serial.println("DS2480B_Detect: success");
    return true;
}

bool DS2480BInterface::OWReset()
{
    _ensureCommandMode();
    _flushRXBuffer();

    Serial2.write(_ds2480b_params.commands.reset_cmd);  // reset at standard/flex speed

    uint32_t timeout = millis() + 10;
    while (!Serial2.available())
    {
        if (millis() > timeout)
        {
            Serial.println("OWReset: timeout waiting for response");
            return false;
        }

    }

    uint8_t response = Serial2.read();
    uint8_t reset_bits = response & 0x03;  // bits 1:0 = presence result

    if (reset_bits == ds2480b_default_parameters::RESET_PRESENCE)
    {
        return true;
    }


    return false;
}

int DS2480BInterface::OWWriteByte(uint8_t data)
{
    _ensureDataMode();
    _flushRXBuffer();

    // If data == 0xE3 it must be sent twice
    Serial2.write(data);
    if (data == 0xE3) Serial2.write(data);

    uint32_t timeout = millis() + 10;  // 10ms timeout
    while (!Serial2.available())
    {
        if (millis() > timeout)
        {
            return -1;
        }
    }

    return Serial2.read();
}

int DS2480BInterface::OWReadByte()
{
    return OWWriteByte(0xFF);
}

void DS2480BInterface::_flushRXBuffer()
{
    while (Serial2.available())
    {
        Serial2.read();
    };
}

void DS2480BInterface::_ensureCommandMode()
{
    if (_curr_mode != DS2480B_Mode::COMMAND_MODE)
    {
        Serial2.write(_ds2480b_params.commands.set_cmd_mode);
        _curr_mode = DS2480B_Mode::COMMAND_MODE;
        delay(2);
        _flushRXBuffer();
    }
}

void DS2480BInterface::_ensureDataMode()
{
    if (_curr_mode != DS2480B_Mode::DATA_MODE)
    {
        Serial2.write(_ds2480b_params.commands.set_data_mode);
        _curr_mode = DS2480B_Mode::DATA_MODE;
        delay(2); // Don't flush bc in data mode we are constantly getting responses.
    }
}