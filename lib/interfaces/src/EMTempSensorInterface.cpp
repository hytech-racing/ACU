#include "EMTempSensorInterface.h"

bool EMTempSensorInterface::_startTempConversion()
{
    if (_DS2480B.OWReset() != ds2480b_default_parameters::RESET_PRESENCE)
    {
        return false;
    }

    _DS2480B.OWWriteByte(_EMtemp_params.commands.skip_rom);
    _DS2480B.OWWriteByte(_EMtemp_params.commands.convert);

    return true;
}

bool EMTempSensorInterface::_ReadTemperature(const ROMID_t& ROM_ID, celsius temperature_c)
{
    // if (!OWMatchROM(ROM_ID.data()))
    //     return false;

    _DS2480B.OWWriteByte(_EMtemp_params.commands.read_scratchpad);

    uint8_t scratchpad[9];

    for (int i = 0; i < 9; i++)
    {
        int b = OWReadByte();

        if (b < 0)
            return false;

        scratchpad[i] = static_cast<uint8_t>(b);
    }

    if (!_CheckCRC(scratchpad, 9))
    {
        Serial.println("Scratchpad CRC failed");
        return false;
    }

    int16_t raw =
        ((int16_t)scratchpad[1] << 8) |
         scratchpad[0];

    temperatureC = raw / 16.0f;

    return true;
}

bool EMTempSensorInterface::_OWMatchROM(const uint8_t ROM_IDs[8])
{
    _DS2480B.OWWriteByte(_EMtemp_params.commands.match_rom);

    for (uint8_t byte : ROM_IDs)
    {
        OWWriteByte(ROM_IDs[id]);
    }

    return true;
}


bool CheckCRC(const uint8_t* data, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        for (int j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;

            if (mix)
                crc ^= 0x8C;

            byte >>= 1;
        }
    }

    return crc == 0;
}





// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
EMTempSensorInterface::EMTempSensorInterface(DS2480B_Teensy&             bus,
                                             const EMTempSensorParams_s& params)
    : _bus(bus),
      _params(params),
      _state(State::IDLE),
      _conversion_start_ms(0)
{
    for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
    {
        _temperatures[i] = 0.0f;
        _read_ok[i]      = false;
        _sensor_ready[i] = false;
    }
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void EMTempSensorInterface::init(uint32_t init_millis)
{
    _state = State::IDLE;

    // Start the first conversion immediately so sensors are ready as soon as
    // the conversion window elapses, rather than waiting one full cycle first.
    if (_start_conversion_all())
    {
        _conversion_start_ms = init_millis;
        _state = State::CONVERTING;
    }
}

// ---------------------------------------------------------------------------
// tick
// Call every loop iteration.  Advances the non-blocking state machine.
// ---------------------------------------------------------------------------
void EMTempSensorInterface::tick(uint32_t curr_millis)
{
    switch (_state)
    {
        case State::IDLE:
        {
            if (_start_conversion_all())
            {
                _conversion_start_ms = curr_millis;
                _state = State::CONVERTING;
            }
            break;
        }

        case State::CONVERTING:
        {
            if ((curr_millis - _conversion_start_ms) >= _params.conversion_time_ms)
            {
                _state = State::READING;
            }
            break;
        }

        case State::READING:
        {
            for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
            {
                _read_sensor(i);
            }
            _state = State::IDLE;
            break;
        }

        default:
        {
            _state = State::IDLE;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// get_temperature
// ---------------------------------------------------------------------------
celsius EMTempSensorInterface::get_temperature(uint8_t sensor_index) const
{
    if (sensor_index >= NUM_EM_TEMP_SENSORS) return 0.0f;
    return _temperatures[sensor_index];
}

// ---------------------------------------------------------------------------
// get_max_temperature
// ---------------------------------------------------------------------------
celsius EMTempSensorInterface::get_max_temperature() const
{
    celsius max_temp = _temperatures[0];

    for (uint8_t i = 1; i < NUM_EM_TEMP_SENSORS; i++)
    {
        if (_sensor_ready[i] && _temperatures[i] > max_temp)
            max_temp = _temperatures[i];
    }

    return max_temp;
}

// ---------------------------------------------------------------------------
// is_overtemp
// ---------------------------------------------------------------------------
bool EMTempSensorInterface::is_overtemp() const
{
    for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
    {
        if (_sensor_ready[i] && (_temperatures[i] > _params.overtemp_threshold))
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// all_sensors_ready
// ---------------------------------------------------------------------------
bool EMTempSensorInterface::all_sensors_ready() const
{
    for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
    {
        if (!_sensor_ready[i]) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// sensor_read_ok
// ---------------------------------------------------------------------------
bool EMTempSensorInterface::sensor_read_ok(uint8_t sensor_index) const
{
    if (sensor_index >= NUM_EM_TEMP_SENSORS) return false;
    return _read_ok[sensor_index];
}

// ---------------------------------------------------------------------------
// get_params
// ---------------------------------------------------------------------------
const EMTempSensorParams_s& EMTempSensorInterface::get_params() const
{
    return _params;
}

// ---------------------------------------------------------------------------
// _start_conversion_all (private)
//
// Sends Skip ROM (0xCC) followed by Convert T (0x44) so all sensors on the
// bus begin conversion simultaneously.  This saves ~4.5 s vs. addressing
// each sensor individually.
// ---------------------------------------------------------------------------
bool EMTempSensorInterface::_start_conversion_all()
{
    if (!_bus.reset())
    {
        Serial.println("BUS RESET IN CONVERSION FAILED");
        return false;
    }

    _bus.skip();
    _bus.write(DS18B20_CONVERT_T);

    return true;
}

// ---------------------------------------------------------------------------
// _read_sensor (private)
//
// Addresses sensor_index by its ROM ID, issues Read Scratchpad, reads 9
// bytes, verifies CRC, range-checks the converted value, and if everything
// passes updates the public temperature array.
// ---------------------------------------------------------------------------
bool EMTempSensorInterface::_read_sensor(uint8_t sensor_index)
{
    if (sensor_index >= NUM_EM_TEMP_SENSORS) return false;

    if (!_bus.reset())
    {
        Serial.print("SENSOR INDEX "); Serial.print(sensor_index); Serial.println(" FAILED TO RESET");
        _read_ok[sensor_index] = false;
        return false;
    }

    _bus.select(_sensor_ids[sensor_index]);
    _bus.write(DS18B20_READ_SCRATCHPAD);

    uint8_t scratchpad[DS18B20_SCRATCHPAD_BYTES];
    _bus.read_bytes(scratchpad, DS18B20_SCRATCHPAD_BYTES);

    // CRC covers bytes 0–7; byte 8 is the CRC itself.
    uint8_t computed_crc = DS2480B_Teensy::crc8(scratchpad, DS18B20_SCRATCHPAD_BYTES - 1);
    if (computed_crc != scratchpad[DS18B20_SCRATCHPAD_BYTES - 1])
    {
        _read_ok[sensor_index] = false;
        return false;
    }

    celsius temp = _raw_to_celsius(scratchpad[0], scratchpad[1]);

    // Reject the DS18B20 power-on default (85 °C) and any other out-of-range
    // value that somehow passed CRC.
    if (temp < _params.min_valid_temp || temp > _params.max_valid_temp)
    {
        _read_ok[sensor_index] = false;
        return false;
    }

    _temperatures[sensor_index] = temp;
    _read_ok[sensor_index]      = true;
    _sensor_ready[sensor_index] = true;
    return true;
}

// ---------------------------------------------------------------------------
// _raw_to_celsius (private)
//
// The DS18B20 stores temperature as a signed 16-bit value in Q12.4 format:
//   bits 15–4 = integer part (two's complement)
//   bits  3–0 = fractional part (1/16 °C per LSB)
// ---------------------------------------------------------------------------
celsius EMTempSensorInterface::_raw_to_celsius(uint8_t lsb, uint8_t msb) const
{
    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
    return static_cast<celsius>(raw) / 16.0f;
}


EMTempData_s EMTempSensorInterface::getCurrentData()
{
    return _curr_data;
}