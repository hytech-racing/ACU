#ifndef EM_TEMP_SENSOR_INTERFACE_H
#define EM_TEMP_SENSOR_INTERFACE_H

#include <inttypes.h>
#include <array>
#include "etl/singleton.h"
#include "DS2480BInterface.h"
#include "SharedFirmwareTypes.h"

 using ROMID_t = std::array<uint8_t, 8>;
namespace EMtemp_default_parameters
{
    constexpr const uint16_t CONVERSION_TIME_MS = 800; // conversion time at 12-bit resolution  maximum is 750 ms; add a small margin.
    constexpr const uint8_t NUM_TEMP_SENSORS = 6;

    constexpr const celsius MIN_VALID_TEMP_C = -10.0f;
    constexpr const celsius MAX_VALID_TEMP_C = 85.0f;
    constexpr const celsius OVERTEMP_C = 60.0f;

    // DS18B20 ROM Commands
    constexpr const uint8_t SEARCH_ROM = 0xF0;
    constexpr const uint8_t READ_ROM = 0x33;
    constexpr const uint8_t MATCH_ROM = 0x55;
    constexpr const uint8_t SKIP_ROM = 0xCC;
    constexpr const uint8_t ALARM_SEARCH = 0xEC;

    // DS18B20 Function Commands
    constexpr const uint8_t CONVERT = 0x44;          // Initiates a single temperature conversion. Resulting data is stored in the 2-byte temperature register
    constexpr const uint8_t WRITE_SCRATCHPAD = 0x4E;  // LSB. All three bytes MUST be written before the master issues a reset
    constexpr const uint8_t READ_SCRATCHPAD = 0xBE;
    constexpr const uint8_t COPY_SCRATCHPAD = 0x48;   // Copies ontents of bytes 2, 3 and 4 to EEPROM
    constexpr const uint8_t SCRATCHPAD_BYTES = 9;     // bytes 0–7 = data, byte 8 = CRC


    // All ROM ID's
    constexpr ROMID_t SENSOR_0 = { 0x28, 0xE0, 0xF4, 0x70, 0x11, 0x00, 0x00, 0xC7 };
    constexpr ROMID_t SENSOR_1 = { 0x28, 0xC8, 0x92, 0x70, 0x11, 0x00, 0x00, 0x5D };
    constexpr ROMID_t SENSOR_2 = { 0x28, 0xA6, 0xF5, 0x10, 0x11, 0x00, 0x00, 0x5D };
    constexpr ROMID_t SENSOR_3 = { 0x28, 0xF9, 0x5A, 0x71, 0x11, 0x00, 0x00, 0x14 };
    constexpr ROMID_t SENSOR_4 = { 0x28, 0x75, 0x42, 0x11, 0x11, 0x00, 0x00, 0x51 };
    constexpr ROMID_t SENSOR_5 = { 0x28, 0x6B, 0xCE, 0x70, 0x11, 0x00, 0x00, 0xEC };

    constexpr std::array<ROMID_t, NUM_TEMP_SENSORS> ALL_SENSORS = {{ SENSOR_0, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5 }};
};

struct EMTempCommands_s
{
    const uint8_t search_rom;
    const uint8_t read_rom;
    const uint8_t match_rom;
    const uint8_t skip_rom;
    const uint8_t alarm_search;
    const uint8_t convert;
    const uint8_t write_scratchpad;
    const uint8_t read_scratchpad;
    const uint8_t copy_scratchpad;
};

struct EMTempThresholds_s
{
    const celsius min_valid_temp_c;
    const celsius max_valid_temp_c;
    const celsius overtemp_c;
};

struct EMTempSensorData_s
{
    std::array<celsius, EMtemp_default_parameters::NUM_TEMP_SENSORS> all_temp_data;
};

struct EMTempParams_s
{
    EMTempCommands_s commands;
    EMTempThresholds_s thresholds;
    std::array<ROMID_t, EMtemp_default_parameters::NUM_TEMP_SENSORS> sensor_rom_ids;
};

class EMTempSensorInterface
{
public:

    EMTempSensorInterface(DS2480BInterface& DS2480B,
                        EMTempCommands_s commands = {
                            .search_rom = EMtemp_default_parameters::SEARCH_ROM,
                            .read_rom = EMtemp_default_parameters::READ_ROM,
                            .match_rom = EMtemp_default_parameters::MATCH_ROM,
                            .skip_rom = EMtemp_default_parameters::SKIP_ROM,
                            .alarm_search = EMtemp_default_parameters::ALARM_SEARCH,
                            .convert = EMtemp_default_parameters::CONVERT,
                            .write_scratchpad = EMtemp_default_parameters::WRITE_SCRATCHPAD,
                            .read_scratchpad = EMtemp_default_parameters::READ_SCRATCHPAD,
                            .copy_scratchpad = EMtemp_default_parameters::COPY_SCRATCHPAD
                        },
                        EMTempThresholds_s thresholds = {
                            .min_valid_temp_c = EMtemp_default_parameters::MIN_VALID_TEMP_C,
                            .max_valid_temp_c = EMtemp_default_parameters::MAX_VALID_TEMP_C,
                            .overtemp_c = EMtemp_default_parameters::OVERTEMP_C
                        },
                        std::array<ROMID_t, EMtemp_default_parameters::NUM_TEMP_SENSORS> all_sensor_rom_ids = EMtemp_default_parameters::ALL_SENSORS
        ) : _DS2480B(DS2480B),
            _params {
                commands,
                thresholds,
                all_sensor_rom_ids
            }
        {
            _curr_data.all_temp_data.fill(NAN);
        }


    void init();

    // Call every loop iteration.  Drives the internal state machine;
    // never blocks for more than a few microseconds per call.
    void tick(uint32_t curr_millis);

    /**
     *  Returns last valid temperature for a sensor, or NAN if not yet read
     */
    celsius get_temperature(uint8_t sensor_index) const;

    /**
     * @return highest temperature across 6 sensors
     */
    celsius get_max_temperature() const;

    /**
     * @return true if any sensor exceeds the overtemp threshold
     */
    bool is_overtemp() const;

    /**
     * Read-only access to current temp data. Does this need to be read only??
     */
    const EMTempSensorData_s& get_current_data() const;

private:
    DS2480BInterface& _DS2480B;
    EMTempParams_s _params;
    EMTempSensorData_s _curr_data;

    enum class State : uint8_t
    {
        IDLE,           // ready to start a new conversion cycle
        CONVERTING,     // waiting for DS18B20 conversion to complete
        READING         // reading scratchpad registers in sequence
    };

    State _state = State::IDLE;
    uint32_t _conversion_start_ms = 0;

    /**
     *
     */
    bool _StartAllTempConversions();

    /**
     * @brief Reads and converts temperature from a sensor. Addresses the sensor by ROM ID, reads its 9-byte scratchpad, validates
     * the CRC, then converts the raw bits to celsius
     *
     * @return true if the temperature can be read and CRC valid
     */
    bool _ReadOneTemperature(uint8_t sensor_index);

    /**
     * @brief matches the ROM ID
     */
    bool _OWMatchROM(const ROMID_t& rom_id);


    /**
     * Standard CRC used by 1-Wire devices is the Dallas/Maxim CRC-8 algorithm
     * This method runs the CRC-8 algorithm.
     * The DS18B20 appends a precomputed CRC as the final byte of its scratchpad.
     * When the algorithm processes all data bytes + appended CRC, a valid
     * transmission will have a remainder of zero. Any corrupted bit will
     * produce a nonzero remainder.
     *
     * Polynomial: x^8 + x^5 + x^4 + 1 -> 1000 1100
     * Taps at register positions: 7, 3, 2
     *
     * @param data is a pointer to byte array which includes the appended CRC byte
     * @param num_bytes is total number of bytes to process (data + CRC byte). Should be 9 for a full "scratchpad"
     * @return true if the CRC remainder is zero (data is valid). If data is invalid, then the algo will not return zero
     */
    bool _CheckCRC(const uint8_t* data, uint8_t num_bytes);

};

using EMTempSensorInterfaceInstance = etl::singleton<EMTempSensorInterface>;

#endif // EM_TEMP_SENSOR_INTERFACE_H