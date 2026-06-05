#ifndef EM_TEMP_SENSOR_INTERFACE_H
#define EM_TEMP_SENSOR_INTERFACE_H

#include <inttypes.h>
#include "etl/singleton.h"
#include "DS2480BInterface.h"
#include "SharedFirmwareTypes.h"

 using ROMID_t = std::array<uint8_t, 8>;
namespace EMtemp_default_parameters
{
    constexpr const uint16_t CONVERSION_TIME_MS = 800; // conversion time at 12-bit resolution  maximum is 750 ms; add a small margin.
    constexpr const uint8_t NUM_TEMP_SENSORS = 6;

    constexpr const celsius DEFAULT_MIN_VALID_TEMP_C = -10.0f;
    constexpr const celsius DEFAULT_MAX_VALID_TEMP_C = 85.0f;
    constexpr const celsius DEFAULT_OVERTEMP_C = 60.0f;

    // DS18B20 ROM Commands
    constexpr const uint8_t SEARCH_ROM = 0xF0;
    constexpr const uint8_t READ_ROM = 0x33;
    constexpr const uint8_t MATCH_ROM = 0x55;
    constexpr const uint8_t SKIP_ROM = 0xCC;
    constexpr const uint8_t ALARM_SEARCH = 0xEC;

    // DS18B20 Function Commands
    constexpr const uint8_t CONVERT =  0x44;          // Initiates a single temperature conversion. Resulting data is stored in the 2-byte temperature register
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

    constexpr std::array<ROMID_t, NUM_TEMP_SENSORS> ALL_SENSORS = {{
        SENSOR_0, SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5
    }};
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
    const celsius default_min_valid_temp_c;
    const celsius default_max_valid_temp_c;
    const celsius default_overtemp_c;
};

struct EMTempSensorData_s
{
    celsius temperature_c;
    bool is_sensor_present;
    bool is_crc_valid;
};

struct EMTempData_s
{
    std::array<EMTempSensorData_s, EMtemp_default_parameters::NUM_TEMP_SENSORS> all_sensor_data;
};

struct EMTempParams_s
{
    EMTempCommands_s commands;
    EMTempThresholds_s thresholds;
    std::array<ROMID_t, EMtemp_default_parameters::NUM_TEMP_SENSORS> all_sensor_rom_ids;
};

class EMTempSensorInterface
{
public:

    EMTempSensorInterface(EMTempCommands_s commands = {
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
                        .default_min_valid_temp_c = EMtemp_default_parameters::DEFAULT_MIN_VALID_TEMP_C,
                        .default_max_valid_temp_c = EMtemp_default_parameters::DEFAULT_MAX_VALID_TEMP_C,
                        .default_overtemp_c = EMtemp_default_parameters::DEFAULT_OVERTEMP_C
                    },
                    std::array<ROMID_t, EMtemp_default_parameters::NUM_TEMP_SENSORS> all_sensor_rom_ids = EMtemp_default_parameters::ALL_SENSORS
    ) : _EMtemp_params {
            commands,
            thresholds,
            all_sensor_rom_ids
        }
    {}

    // Call once in setup() after bus.begin().  Kicks off the first conversion.
    void init(uint32_t init_millis);

    // Call every loop iteration.  Drives the internal state machine;
    // never blocks for more than a few microseconds per call.
    void tick(uint32_t curr_millis);

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    // Last accepted temperature for sensor [0 … NUM_EM_TEMP_SENSORS-1].
    // Returns 0.0 if the index is out of range or no valid read has occurred.
    celsius get_temperature(uint8_t sensor_index) const;

    // Highest temperature across all sensors.
    celsius get_max_temperature() const;

    // True if any ready sensor exceeds params.overtemp_threshold.
    bool is_overtemp() const;

    // True once every sensor has returned at least one CRC-valid reading.
    bool all_sensors_ready() const;

    // True if the most recent read of sensor_index passed CRC and range check.
    bool sensor_read_ok(uint8_t sensor_index) const;

    // Read-only access to the active parameter set.
    const EMTempParams_s& get_params() const;


    EMTempData_s getCurrentData();


private:
    EMTempParams_s _EMtemp_params = {};
    EMTempData_s _curr_data;
    DS2480BInterface& _DS2480B;

    bool _startTempConversion();
    bool _ReadTemperature(const ROMID_t& ROM_ID, celsius temperature_c);
    bool _OWMatchROM(const uint8_t ROM_IDs[8]);


    // -----------------------------------------------------------------------
    // Internal state machine
    // -----------------------------------------------------------------------
    enum class State : uint8_t
    {
        IDLE,           // ready to start a new conversion cycle
        CONVERTING,     // waiting for DS18B20 conversion to complete
        READING         // reading scratchpad registers in sequence
    };

    // DS2480B_Teensy&      _bus;
    // EMTempSensorParams_s _params;

    // ROM IDs — order must match physical harness sensor positions 0–5.
    static const uint8_t _sensor_ids[NUM_EM_TEMP_SENSORS][8];

    celsius  _temperatures[NUM_EM_TEMP_SENSORS];
    bool     _read_ok[NUM_EM_TEMP_SENSORS];
    bool     _sensor_ready[NUM_EM_TEMP_SENSORS];

    State    _state;
    uint32_t _conversion_start_ms;

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    // Issue Skip ROM + Convert T to start simultaneous conversion on all sensors.
    // Returns false if the 1-Wire reset found no devices.
    bool _start_conversion_all();

    // Read and validate the scratchpad of one sensor.
    // Updates _temperatures, _read_ok, _sensor_ready on success.
    // Returns false on bus error, CRC mismatch, or out-of-range value.
    bool _read_sensor(uint8_t sensor_index);

    // Convert DS18B20 raw scratchpad bytes 0–1 to degrees Celsius.
    celsius _raw_to_celsius(uint8_t lsb, uint8_t msb) const;
};

using EMTempSensorInterfaceInstance = etl::singleton<EMTempSensorInterface>;

#endif // EM_TEMP_SENSOR_INTERFACE_H