#ifndef EM_TEMP_SENSOR_INTERFACE_H
#define EM_TEMP_SENSOR_INTERFACE_H

#include <inttypes.h>
#include "DS2480B.h"
#include "SharedFirmwareTypes.h"

// ---------------------------------------------------------------------------
// DS18B20 protocol constants
// ---------------------------------------------------------------------------
#define DS18B20_CONVERT_T           0x44
#define DS18B20_READ_SCRATCHPAD     0xBE
#define DS18B20_SCRATCHPAD_BYTES    9   // bytes 0–7 = data, byte 8 = CRC

// DS18B20 conversion time at 12-bit resolution (milliseconds).
// The datasheet maximum is 750 ms; add a small margin.
#define DS18B20_CONVERSION_TIME_MS  800

// ---------------------------------------------------------------------------
// Pack sensor count
// ---------------------------------------------------------------------------
#define NUM_EM_TEMP_SENSORS         6

// ---------------------------------------------------------------------------
// Default thresholds — override via EMTempSensorParams_s at construction
// ---------------------------------------------------------------------------
#define EM_TEMP_DEFAULT_MIN_VALID_C     -10.0f
#define EM_TEMP_DEFAULT_MAX_VALID_C      85.0f
#define EM_TEMP_DEFAULT_OVERTEMP_C       60.0f

// ---------------------------------------------------------------------------
// EMTempSensorParams_s
// All tuneable values in one place; pass a configured instance to the
// constructor.  Defaults are set inline so a zero-initialised struct works.
// ---------------------------------------------------------------------------
struct EMTempSensorParams_s
{
    // Milliseconds to wait after issuing Convert T before reading scratchpad.
    // Must be >= 750 ms for 12-bit resolution.
    uint32_t conversion_time_ms = DS18B20_CONVERSION_TIME_MS;

    // A reading outside [min_valid_temp, max_valid_temp] is rejected even if
    // CRC passes (catches power-on 85 °C default and other anomalies).
    celsius  min_valid_temp     = EM_TEMP_DEFAULT_MIN_VALID_C;
    celsius  max_valid_temp     = EM_TEMP_DEFAULT_MAX_VALID_C;

    // Any sensor reading above this value triggers is_overtemp().
    celsius  overtemp_threshold = EM_TEMP_DEFAULT_OVERTEMP_C;
};

// ---------------------------------------------------------------------------
// EMTempSensorInterface
//
// Non-blocking driver for 6 × DS18B20 temperature sensors connected to a
// single 1-Wire bus via a DS2480B UART bridge on a Teensy hardware serial
// port.
//
// Typical usage:
//   DS2480B_Teensy         bus(Serial1);
//   EMTempSensorParams_s   params;
//   EMTempSensorInterface  tempSensors(bus, params);
//
//   setup() { bus.begin(); tempSensors.init(millis()); }
//   loop()  { tempSensors.tick(millis()); }
// ---------------------------------------------------------------------------
class EMTempSensorInterface
{
public:

    EMTempSensorInterface(DS2480B_Teensy&            bus,
                          const EMTempSensorParams_s& params);

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
    const EMTempSensorParams_s& get_params() const;

private:

    // -----------------------------------------------------------------------
    // Internal state machine
    // -----------------------------------------------------------------------
    enum class State : uint8_t
    {
        IDLE,           // ready to start a new conversion cycle
        CONVERTING,     // waiting for DS18B20 conversion to complete
        READING         // reading scratchpad registers in sequence
    };

    DS2480B_Teensy&      _bus;
    EMTempSensorParams_s _params;

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

#endif // EM_TEMP_SENSOR_INTERFACE_H