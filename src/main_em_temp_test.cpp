#include <Arduino.h>

#include "DS2480B.h"
#include "EMTempSensorInterface.h"

// =============================================================================
// main.cpp
//
// Entry point for the EM temperature monitoring subsystem on Teensy 4.x.
//
// Wiring:
//   DS2480B TX  →  Teensy Serial1 RX  (pin 0)
//   DS2480B RX  →  Teensy Serial1 TX  (pin 1)
//   DS2480B GND →  Teensy GND
//   DS2480B VCC →  3.3 V (check your DS2480B module's regulator rating)
//
// Build system note:
//   Arduino-based build systems (Teensyduino, PlatformIO) automatically call
//   setup() and loop() from their generated main().  If you are compiling
//   outside that environment, replace setup()/loop() with a standard main()
//   and insert a while(1) loop around the loop() body.
// =============================================================================

// ---------------------------------------------------------------------------
static DS2480B_Teensy bus(Serial2);

// ---------------------------------------------------------------------------
// Sensor configuration
// All fields have defaults in EMTempSensorParams_s; only set what differs.
// ---------------------------------------------------------------------------
static EMTempSensorParams_s temp_params =
{
    .conversion_time_ms = 800,      // >= 750 ms required for 12-bit resolution
    .min_valid_temp     = -10.0f,   // reject anything colder than this
    .max_valid_temp     =  85.0f,   // also rejects the DS18B20 power-on default
    .overtemp_threshold =  60.0f    // tune to your cell / pack specification
};

static EMTempSensorInterface temp_sensors(bus, temp_params);

// ---------------------------------------------------------------------------
// Debug serial print interval (milliseconds)
// ---------------------------------------------------------------------------
static constexpr uint32_t PRINT_INTERVAL_MS = 1000;

// =============================================================================
// setup
// =============================================================================
void setup()
{
    // USB CDC serial — used for debug output only.
    // Remove or guard with #ifdef once integrated into the full BMS firmware.
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait up to 3 s for USB enumeration

    // Initialise the DS2480B UART bridge, then start the first conversion.
    bus.begin();
    temp_sensors.init(millis());

    Serial.println("[INIT] EMTempSensorInterface ready");
}

// =============================================================================
// loop
// =============================================================================
void loop()
{
    const uint32_t now = millis();

    // Drive the non-blocking state machine.  Must be called every iteration.
    temp_sensors.tick(now);

    // -------------------------------------------------------------------------
    // Fault check — evaluate on every tick so the response latency is bounded
    // by loop() execution time, not the print interval.
    // -------------------------------------------------------------------------
    if (temp_sensors.is_overtemp())
    {
        // TODO: assert BMS fault line / trigger shutdown state machine here.
        // This stub just prints; replace with your fault handler call.
        Serial.println("[FAULT] OVERTEMP detected");
    }

    // -------------------------------------------------------------------------
    // Periodic debug output
    // -------------------------------------------------------------------------
    static uint32_t last_print_ms = 0;

    if ((now - last_print_ms) >= PRINT_INTERVAL_MS)
    {
        last_print_ms = now;

        if (!temp_sensors.all_sensors_ready())
        {
            Serial.println("[TEMP] Waiting for first valid read on all sensors...");
            return;
        }

        Serial.println("[TEMP] --- Pack temperatures ---");

        for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
        {
            Serial.print("  Sensor ");
            Serial.print(i);
            Serial.print(" : ");

            if (temp_sensors.sensor_read_ok(i))
            {
                Serial.print(temp_sensors.get_temperature(i), 2);
                Serial.println(" C");
            }
            else
            {
                Serial.println("ERROR  (CRC fail or out-of-range)");
            }
        }

        Serial.print("  Max     : ");
        Serial.print(temp_sensors.get_max_temperature(), 2);
        Serial.println(" C");
        Serial.println();
    }
}