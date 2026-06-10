#include "EMTempSensorInterface.h"

void EMTempSensorInterface::init()
{
    _DS2480B.init();

    _state = State::IDLE;
    _curr_sensor_index = 0;
    _curr_micros = 0;
    _conversion_start_us = 0;
    _curr_data.all_temp_data.fill(NAN);
}

// void EMTempSensorInterface::tick(uint32_t curr_millis)
// {
//     (void)curr_millis;

//     for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
//     {
//         _read_one_temperature(i);
//     }
// }

void EMTempSensorInterface::tick(uint32_t curr_micros)
{
    _curr_micros = curr_micros;

    switch (_state)
    {
        case State::IDLE:
        {
            _handle_idle();
            break;
        }
        case State::CONVERTING:
        {
            _handle_converting();
            break;
        }
        case State::READING:
        {
            _handle_reading();
            break;
        }
    }
}

celsius EMTempSensorInterface::get_temperature(uint8_t sensor_index) const
{
    if (sensor_index >= EMtemp_default_parameters::NUM_TEMP_SENSORS)
    {
        return NAN;
    }

    return _curr_data.all_temp_data[sensor_index];
}

celsius EMTempSensorInterface::get_max_temperature() const
{
    celsius max_temp = NAN;

    for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
    {
        if (!isnan(_curr_data.all_temp_data[i]))
        {
            if (isnan(max_temp) || _curr_data.all_temp_data[i] > max_temp)
            {
                max_temp = _curr_data.all_temp_data[i];
            }
        }
    }

    return max_temp;
}

uint8_t EMTempSensorInterface::get_current_sensor_index() const
{
    return _curr_sensor_index;
}

bool EMTempSensorInterface::is_overtemp() const
{
    for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
    {
        if (!isnan(_curr_data.all_temp_data[i]) && _curr_data.all_temp_data[i] > _params.thresholds.overtemp_c)
        {
            return true;
        }
    }
    return false;
}

const EMTempSensorData_s& EMTempSensorInterface::get_current_data() const
{
    return _curr_data;
}

// bool EMTempSensorInterface::_start_all_temp_conversions()
// {
//     if (!_DS2480B.OWReset())
//     {
//         return false;
//     }


//     Serial.println("START CONVERT");

//     int resp;

//     resp = _DS2480B.OWWriteByte(_params.commands.skip_rom);

//     Serial.print("SKIP_ROM = 0x");
//     Serial.println(resp, HEX);

//     resp = _DS2480B.OWWriteByte(_params.commands.convert);

//     Serial.print("CONVERT = 0x");
//     Serial.println(resp, HEX);

//     // _DS2480B.OWWriteByte(_params.commands.skip_rom);
//     // _DS2480B.OWWriteByte(_params.commands.convert);

//     return true;
// }
// bool EMTempSensorInterface::_start_all_temp_conversions()
// {
//     const ROMID_t& rom_id = _params.sensor_rom_ids[0];

//     if (!_OWMatchROM(rom_id))
//     {
//         return false;
//     }

//     int resp = _DS2480B.OWWriteByte(_params.commands.convert);

//     Serial.print("CONVERT response = 0x");
//     Serial.println(resp, HEX);

//     return true;
// }

void EMTempSensorInterface::_handle_idle()
{
    const ROMID_t& rom_id = _params.sensor_rom_ids[_curr_sensor_index];

    if (!_OWMatchROM(rom_id))
    {
        _curr_data.all_temp_data[_curr_sensor_index] = NAN;
        _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;
        _state = State::IDLE;
        return;
    }

    _DS2480B.OWWriteByte(_params.commands.convert);
    _conversion_start_us = micros();   // capture AFTER the convert command completes
    _state = State::CONVERTING;
}


void EMTempSensorInterface::_handle_converting()
{
    if ((micros() - _conversion_start_us) >= EMtemp_default_parameters::CONVERSION_TIME_US)
    {
        _state = State::READING;
    }
}

void EMTempSensorInterface::_handle_reading()
{
    // Serial.print("Elapsed us: ");
    // Serial.println(_curr_micros - _conversion_start_us);

    const ROMID_t& rom_id = _params.sensor_rom_ids[_curr_sensor_index];

    if (!_OWMatchROM(rom_id))
    {
        _curr_data.all_temp_data[_curr_sensor_index] = NAN;
        _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;
        _state = State::IDLE;
        return;
    }

    _DS2480B.OWWriteByte(_params.commands.read_scratchpad);

    uint8_t scratchpad[EMtemp_default_parameters::SCRATCHPAD_BYTES];

    if (!_read_scratchpad(scratchpad))
    {
        _curr_data.all_temp_data[_curr_sensor_index] = NAN;
        _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;
        _state = State::IDLE;
        return;
    }

    if (!_CheckCRC(scratchpad, EMtemp_default_parameters::SCRATCHPAD_BYTES))
    {
        _curr_data.all_temp_data[_curr_sensor_index] = NAN;
        _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;
        _state = State::IDLE;
        return;
    }

    // _curr_data.all_temp_data[_curr_sensor_index] = _parse_temperature(scratchpad);
    // _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;
    // _state = State::IDLE;

    celsius temp = _parse_temperature(scratchpad);

    Serial.print("[TEMP UPDATE] Sensor ");
    Serial.print(_curr_sensor_index);

    Serial.print(" RawBytes=");
    if (scratchpad[1] < 0x10) Serial.print('0');
    Serial.print(scratchpad[1], HEX);

    Serial.print(" ");
    if (scratchpad[0] < 0x10) Serial.print('0');
    Serial.print(scratchpad[0], HEX);

    Serial.print(" Temp=");
    Serial.print(temp, 4);

    Serial.print(" C @ ");
    Serial.print(millis());
    Serial.println(" ms");

    _curr_data.all_temp_data[_curr_sensor_index] = temp;

    _curr_sensor_index = (_curr_sensor_index + 1) % EMtemp_default_parameters::NUM_TEMP_SENSORS;

    _state = State::IDLE;
}

bool EMTempSensorInterface::_read_scratchpad(uint8_t* scratchpad)
{
    for (uint8_t i = 0; i < EMtemp_default_parameters::SCRATCHPAD_BYTES; i++)
    {
        int received_byte = _DS2480B.OWReadByte();

        if (received_byte < 0)
        {
            return false;
        }

        scratchpad[i] = static_cast<uint8_t>(received_byte);
    }

    // Serial.print("Scratchpad: ");
    // for (uint8_t i = 0; i < EMtemp_default_parameters::SCRATCHPAD_BYTES; i++)
    // {
    //     if (scratchpad[i] < 0x10) Serial.print('0');
    //     Serial.print(scratchpad[i], HEX);
    //     Serial.print(' ');
    // }
    // Serial.println();

    return true;
}

celsius EMTempSensorInterface::_parse_temperature(const uint8_t* scratchpad)
{
    int16_t raw    = (static_cast<int16_t>(scratchpad[1]) << 8) | scratchpad[0];
    celsius temp_c = raw / 16.0f;

    if (temp_c < _params.thresholds.min_valid_temp_c ||
        temp_c > _params.thresholds.max_valid_temp_c)
    {
        return NAN;
    }

    return temp_c;
}

// bool EMTempSensorInterface::_read_one_temperature(uint8_t sensor_index)
// {
//     if (sensor_index >= EMtemp_default_parameters::NUM_TEMP_SENSORS)
//     {
//         return false;
//     }

//     const ROMID_t& rom_id = _params.sensor_rom_ids[sensor_index];

//     if (!_OWMatchROM(rom_id))
//     {
//         _curr_data.all_temp_data[sensor_index] = NAN;
//         return false;
//     }

//     int response = _DS2480B.OWWriteByte(_params.commands.convert);

//     // Serial.print("Sensor ");
//     // Serial.print(sensor_index);
//     // Serial.print(" convert resp = 0x");
//     // Serial.println(response, HEX);

//     delay(1000);

//     if (!_OWMatchROM(rom_id))
//     {
//         _curr_data.all_temp_data[sensor_index] = NAN;
//         return false;
//     }

//     _DS2480B.OWWriteByte(_params.commands.read_scratchpad);

//     uint8_t scratchpad[EMtemp_default_parameters::SCRATCHPAD_BYTES];

//     for (uint8_t i = 0; i < EMtemp_default_parameters::SCRATCHPAD_BYTES; i++) // for each byte of the 9 byte scratchpad, we are going to read it and then fill it in the array
//     {
//         int recieved_byte = _DS2480B.OWReadByte();

//         if (recieved_byte < 0)
//         {
//             _curr_data.all_temp_data[sensor_index] = NAN;
//             return false;
//         }

//         scratchpad[i] = static_cast<uint8_t>(recieved_byte);
//     }

//     Serial.print("Scratchpad: ");

//     for (uint8_t i = 0; i < 9; i++)
//     {
//         if (scratchpad[i] < 0x10)
//             Serial.print('0');

//         Serial.print(scratchpad[i], HEX);
//         Serial.print(' ');
//     }

//     Serial.println();

//     if (!_CheckCRC(scratchpad, EMtemp_default_parameters::SCRATCHPAD_BYTES))
//     {
//         _curr_data.all_temp_data[sensor_index] = NAN;
//         return false;
//     }

//     int16_t raw = (static_cast<int16_t>(scratchpad[1]) << 8) | scratchpad[0]; // 2 temp bytes, the bottom 4 bits are the fractional parts
//     celsius temp_c = raw / 16.0f;

//     if (temp_c < _params.thresholds.min_valid_temp_c || temp_c > _params.thresholds.max_valid_temp_c)
//     {
//         _curr_data.all_temp_data[sensor_index] = NAN;
//         return false;
//     }

//     _curr_data.all_temp_data[sensor_index] = temp_c;
//     return true;
// }

bool EMTempSensorInterface::_OWMatchROM(const ROMID_t& rom_id)
{
    if (!_DS2480B.OWReset())
    {
        return false;
    }

    _DS2480B.OWWriteByte(_params.commands.match_rom);

    for (uint8_t byte : rom_id)
    {
        _DS2480B.OWWriteByte(byte);
    }

    return true;
}

bool EMTempSensorInterface::_CheckCRC(const uint8_t* data, uint8_t num_bytes)
{
    uint8_t remainder = 0;

    for (uint8_t i = 0; i < num_bytes; i++) // "i" is the byte index
    {
        uint8_t curr_byte = data[i];

        for (uint8_t j = 0; j < 8; j++) // "j" is the bit index
        {
            uint8_t XOR_result = (remainder ^ curr_byte) & 0x01;
            remainder >>= 1; // Shift register, rightmost bit "falls" off

            if (XOR_result) // then we are going to flip the "taps"
            {
                remainder ^= 0x8C;
            }

            curr_byte >>= 1;
        }
    }

    return remainder == 0;
}