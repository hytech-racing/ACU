// #include <Arduino.h>

// #include "DS2480B.h"
// #include "EMTempSensorInterface.h"

// // ---------------------------------------------------------------------------
// static DS2480B_Teensy bus(Serial2);

// // ---------------------------------------------------------------------------
// // Sensor configuration
// // All fields have defaults in EMTempSensorParams_s; only set what differs.
// // ---------------------------------------------------------------------------
// static EMTempSensorParams_s temp_params =
// {
//     .conversion_time_ms = 800,      // >= 750 ms required for 12-bit resolution
//     .min_valid_temp     = -10.0f,   // reject anything colder than this
//     .max_valid_temp     =  85.0f,   // also rejects the DS18B20 power-on default
//     .overtemp_threshold =  60.0f    // tune to your cell / pack specification
// };

// static EMTempSensorInterface temp_sensors(bus, temp_params);

// // ---------------------------------------------------------------------------
// // Debug serial print interval (milliseconds)
// // ---------------------------------------------------------------------------
// static constexpr uint32_t PRINT_INTERVAL_MS = 1000;

// // =============================================================================
// // setup
// // =============================================================================
// void setup()
// {
//     // USB CDC serial — used for debug output only.
//     // Remove or guard with #ifdef once integrated into the full BMS firmware.
//     Serial.begin(115200);
//     while (!Serial && millis() < 3000) {}   // wait up to 3 s for USB enumeration

//     // Initialise the DS2480B UART bridge, then start the first conversion.
//     bus.begin();
//     temp_sensors.init(millis());

//     Serial.println("[INIT] EMTempSensorInterface ready");
// }

// // =============================================================================
// // loop
// // =============================================================================
// void loop()
// {
//     const uint32_t now = millis();

//     // Drive the non-blocking state machine.  Must be called every iteration.
//     temp_sensors.tick(now);

//     // -------------------------------------------------------------------------
//     // Fault check — evaluate on every tick so the response latency is bounded
//     // by loop() execution time, not the print interval.
//     // -------------------------------------------------------------------------
//     if (temp_sensors.is_overtemp())
//     {
//         // TODO: assert BMS fault line / trigger shutdown state machine here.
//         // This stub just prints; replace with your fault handler call.
//         Serial.println("[FAULT] OVERTEMP detected");
//     }

//     // -------------------------------------------------------------------------
//     // Periodic debug output
//     // -------------------------------------------------------------------------
//     static uint32_t last_print_ms = 0;

//     if ((now - last_print_ms) >= PRINT_INTERVAL_MS)
//     {
//         last_print_ms = now;

//         if (!temp_sensors.all_sensors_ready())
//         {
//             Serial.println("[TEMP] Waiting for first valid read on all sensors...");
//             return;
//         }

//         Serial.println("[TEMP] --- Pack temperatures ---");

//         for (uint8_t i = 0; i < NUM_EM_TEMP_SENSORS; i++)
//         {
//             Serial.print("  Sensor ");
//             Serial.print(i);
//             Serial.print(" : ");

//             if (temp_sensors.sensor_read_ok(i))
//             {
//                 Serial.print(temp_sensors.get_temperature(i), 2);
//                 Serial.println(" C");
//             }
//             else
//             {
//                 Serial.println("ERROR  (CRC fail or out-of-range)");
//             }
//         }

//         Serial.print("  Max     : ");
//         Serial.print(temp_sensors.get_max_temperature(), 2);
//         Serial.println(" C");
//         Serial.println();
//     }
// }

#include <Arduino.h>
#include "DS2480B.h"

// =============================================================================
// DS2480B + DS18B20 — Teensy hardware serial test
//
// Wiring:
//   DS2480B TX  →  Teensy Serial1 RX  (pin 0)
//   DS2480B RX  →  Teensy Serial1 TX  (pin 1)
//   DS2480B GND →  Teensy GND
//   DS2480B VCC →  3.3 V
//
// Open Serial Monitor at 115200 baud.
// This will enumerate every sensor on the bus, print its ROM ID, and print
// its temperature in Celsius and Fahrenheit.  Mirrors the original example
// logic but uses HardwareSerial instead of AltSoftSerial.
// =============================================================================

static DS2480B_Teensy ds(Serial2);

// =============================================================================
// setup
// =============================================================================
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait for USB CDC enumeration

    ds.begin();

    Serial.println("[TEST] DS2480B + DS18B20 bus scan started");
    Serial.println("[TEST] Using Serial1 (RX=pin0, TX=pin1) at 9600 baud");
    Serial.println();
}

// =============================================================================
// loop
// =============================================================================
void loop()
{
    uint8_t addr[8];
    uint8_t data[9];

    // -------------------------------------------------------------------------
    // Search for the next device on the bus.
    // search() advances an internal cursor; when all devices have been found
    // it returns 0 and reset_search() is called automatically inside DS2480B.
    // -------------------------------------------------------------------------
    if (!ds.search(addr))
    {
        Serial.println("--- No more devices found, restarting scan in 3 s ---");
        Serial.println();
        ds.reset_search();
        delay(3000);
        return;
    }

    // -------------------------------------------------------------------------
    // Print ROM ID
    // -------------------------------------------------------------------------
    Serial.print("ROM ID : ");
    for (uint8_t i = 0; i < 8; i++)
    {
        if (addr[i] < 0x10) Serial.print("0");  // leading zero for readability
        Serial.print(addr[i], HEX);
        if (i < 7) Serial.print(":");
    }
    Serial.println();

    // -------------------------------------------------------------------------
    // Validate ROM CRC (byte 7 must equal CRC of bytes 0–6)
    // -------------------------------------------------------------------------
    if (DS2480B_Teensy::crc8(addr, 7) != addr[7])
    {
        Serial.println("  ERROR: ROM CRC mismatch — skipping device");
        Serial.println();
        return;
    }

    // -------------------------------------------------------------------------
    // Confirm family code is DS18B20 (0x28)
    // -------------------------------------------------------------------------
    if (addr[0] != 0x28)
    {
        Serial.print("  WARNING: unexpected family code 0x");
        Serial.print(addr[0], HEX);
        Serial.println(" — expected 0x28 for DS18B20, continuing anyway");
    }
    else
    {
        Serial.println("  Chip   : DS18B20 confirmed");
    }

    // -------------------------------------------------------------------------
    // Issue Convert T command to this specific sensor
    // -------------------------------------------------------------------------
    ds.reset();
    ds.select(addr);
    ds.write(0x44);     // Convert T — starts temperature measurement

    // Wait for conversion.  DS18B20 at 12-bit resolution takes up to 750 ms.
    // Polling the bus (reading 0 = busy, 1 = done) is cleaner than a fixed
    // delay but requires parasitic power to be off; fixed delay is safer here.
    delay(800);

    // -------------------------------------------------------------------------
    // Read scratchpad
    // -------------------------------------------------------------------------
    uint8_t present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);     // Read Scratchpad

    for (uint8_t i = 0; i < 9; i++)
    {
        data[i] = ds.read();
    }

    // -------------------------------------------------------------------------
    // Validate scratchpad CRC (byte 8 must equal CRC of bytes 0–7)
    // -------------------------------------------------------------------------
    if (DS2480B_Teensy::crc8(data, 8) != data[8])
    {
        Serial.println("  ERROR: scratchpad CRC mismatch — discarding reading");
        Serial.println();
        return;
    }

    // Print raw scratchpad bytes for debugging
    Serial.print("  Scratchpad (hex) : ");
    for (uint8_t i = 0; i < 9; i++)
    {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // -------------------------------------------------------------------------
    // Convert raw bytes to temperature
    //
    // The DS18B20 scratchpad bytes 0–1 hold a Q12.4 signed fixed-point value:
    //   bits 15–4  integer part (two's complement degrees C)
    //   bits  3–0  fractional part (1/16 °C per LSB)
    //
    // The config register (byte 4) bits 5–6 indicate resolution:
    //   0x00 = 9-bit  (~93.75 ms),  mask low 3 bits
    //   0x20 = 10-bit (~187.5 ms),  mask low 2 bits
    //   0x40 = 11-bit (~375 ms),    mask low 1 bit
    //   0x60 = 12-bit (~750 ms),    no masking needed
    // -------------------------------------------------------------------------
    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);

    uint8_t cfg = (data[4] & 0x60);
    if      (cfg == 0x00) raw &= ~0x07;   //  9-bit
    else if (cfg == 0x20) raw &= ~0x03;   // 10-bit
    else if (cfg == 0x40) raw &= ~0x01;   // 11-bit
    // 0x60 = 12-bit, no masking needed

    float celsius    = static_cast<float>(raw) / 16.0f;
    float fahrenheit = celsius * 1.8f + 32.0f;

    Serial.print("  Resolution : ");
    Serial.print(9 + (cfg >> 5));
    Serial.println(" bit");

    Serial.print("  Temperature: ");
    Serial.print(celsius, 4);
    Serial.print(" C  /  ");
    Serial.print(fahrenheit, 4);
    Serial.println(" F");

    Serial.println();
}