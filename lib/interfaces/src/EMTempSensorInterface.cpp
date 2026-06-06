#include "EMTempSensorInterface.h"

void EMTempSensorInterface::init()
{
    _state = State::IDLE;
    _curr_data.all_temp_data.fill(NAN);
}

// void EMTempSensorInterface::tick(uint32_t curr_millis)
// {
//     switch (_state)
//     {
//         case State::IDLE:
//         {
//             if (_StartAllTempConversions())
//             {
//                 _conversion_start_ms = curr_millis;
//                 _state = State::CONVERTING;
//             }
//             break;
//         }

//         case State::CONVERTING:
//         {
//             if ((curr_millis - _conversion_start_ms) >= EMtemp_default_parameters::CONVERSION_TIME_MS)
//                 _state = State::READING;
//             break;
//         }

//         case State::READING:
//         {
//             for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
//                 _ReadOneTemperature(i);

//             _state = State::IDLE;
//             break;
//         }

//         default:
//             _state = State::IDLE;
//             break;
//     }
//}
void EMTempSensorInterface::tick(uint32_t curr_millis)
{
    for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
    {
        _ReadOneTemperature(i);
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

bool EMTempSensorInterface::_StartAllTempConversions()
{
    if (_DS2480B.OWReset() != ds2480b_default_parameters::RESET_PRESENCE)
    {
        return false;
    }

    _DS2480B.OWWriteByte(_params.commands.skip_rom);
    _DS2480B.OWWriteByte(_params.commands.convert);

    return true;
}

bool EMTempSensorInterface::_ReadOneTemperature(uint8_t sensor_index)
{
    const ROMID_t& rom_id = _params.sensor_rom_ids[sensor_index];

    if (sensor_index >= EMtemp_default_parameters::NUM_TEMP_SENSORS)
    {
        return false;
    }

    if (!_OWMatchROM(rom_id))
    {
        _curr_data.all_temp_data[sensor_index] = NAN;
        return false;
    }

    _DS2480B.OWWriteByte(_params.commands.read_scratchpad);
    delay(800);

    uint8_t scratchpad[EMtemp_default_parameters::SCRATCHPAD_BYTES];

    for (uint8_t i = 0; i < EMtemp_default_parameters::SCRATCHPAD_BYTES; i++) // for each byte of the 9 byte scratchpad, we are going to read it and then fill it in the array
    {
        int recieved_byte = _DS2480B.OWReadByte();

        if (recieved_byte < 0)
        {
            _curr_data.all_temp_data[sensor_index] = NAN;
            return false;
        }

        scratchpad[i] = static_cast<uint8_t>(recieved_byte);
    }

    if (!_CheckCRC(scratchpad, EMtemp_default_parameters::SCRATCHPAD_BYTES))
    {
        return false;
    }

    int16_t raw = (static_cast<int16_t>(scratchpad[1]) << 8) | scratchpad[0]; // 2 temp bytes, the bottom 4 bits are the fractional parts
    celsius temp_c = raw / 16.0f;

    if (temp_c < _params.thresholds.min_valid_temp_c ||
        temp_c > _params.thresholds.max_valid_temp_c)
    {
        _curr_data.all_temp_data[sensor_index] = NAN;
        return false;
    }

    _curr_data.all_temp_data[sensor_index] = temp_c;
    return true;
}

bool EMTempSensorInterface::_OWMatchROM(const ROMID_t& rom_id)
{
    if (_DS2480B.OWReset() != ds2480b_default_parameters::RESET_PRESENCE)
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