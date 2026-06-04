#include <Arduino.h>

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

// #include <Arduino.h>
// #include "DS2480B.h"

// // =============================================================================
// // DS2480B + DS18B20 — Teensy hardware serial test
// //
// // Wiring:
// //   DS2480B TX  →  Teensy Serial2 RX  (pin 7)
// //   DS2480B RX  →  Teensy Serial2 TX  (pin 8)
// //   DS2480B GND →  Teensy GND
// //   DS2480B VCC →  3.3 V
// //
// // Open Serial Monitor at 115200 baud.
// // This will enumerate every sensor on the bus, print its ROM ID, and print
// // its temperature in Celsius and Fahrenheit.  Mirrors the original example
// // logic but uses HardwareSerial instead of AltSoftSerial.
// // =============================================================================

// static DS2480B_Teensy ds(Serial2);

// // =============================================================================
// // setup
// // =============================================================================
// void setup()
// {
//     Serial.begin(115200);
//     while (!Serial && millis() < 3000) {}   // wait for USB CDC enumeration

//     ds.begin();

//     Serial.println("[TEST] DS2480B + DS18B20 bus scan started");
//     Serial.println("[TEST] Using Serial2 (RX=pin7, TX=pin8) at 9600 baud");
//     Serial.println();

//     // Verify the DS2480B is alive by issuing one reset and printing the raw
//     // response byte.  Expected: 0xCD (device present) or 0xE3 (no device but
//     // bus OK).  Anything else (0x00, timeout) means a wiring or baud problem.
//     Serial2.write(0xC1);
//     uint32_t t = millis();
//     while (!Serial2.available() && (millis() - t) < 100) {}
//     if (Serial2.available())
//     {
//         uint8_t resp = Serial2.read();
//         Serial.print("[TEST] Raw reset response: 0x");
//         if (resp < 0x10) Serial.print("0");
//         Serial.print(resp, HEX);
//         if      (resp == 0xCD) Serial.println("  -> OK: device(s) present");
//         else if (resp == 0xE3) Serial.println("  -> OK: bus present, no device");
//         else                   Serial.println("  -> UNEXPECTED — check wiring/baud");
//     }
//     else
//     {
//         Serial.println("[TEST] No response to reset — check TX/RX wiring and VCC");
//     }
//     Serial.println();
// }

// // =============================================================================
// // loop
// // =============================================================================
// void loop()
// {
//     uint8_t addr[8];
//     uint8_t data[9];

//     // -------------------------------------------------------------------------
//     // Search for the next device on the bus.
//     // search() advances an internal cursor; when all devices have been found
//     // it returns 0 and reset_search() is called automatically inside DS2480B.
//     // -------------------------------------------------------------------------
//     if (!ds.search(addr))
//     {
//         Serial.println("--- No more devices found, restarting scan in 3 s ---");
//         Serial.println();
//         ds.reset_search();
//         delay(3000);
//         return;
//     }

//     // -------------------------------------------------------------------------
//     // Print ROM ID
//     // -------------------------------------------------------------------------
//     Serial.print("ROM ID : ");
//     for (uint8_t i = 0; i < 8; i++)
//     {
//         if (addr[i] < 0x10) Serial.print("0");  // leading zero for readability
//         Serial.print(addr[i], HEX);
//         if (i < 7) Serial.print(":");
//     }
//     Serial.println();

//     // -------------------------------------------------------------------------
//     // Validate ROM CRC (byte 7 must equal CRC of bytes 0–6)
//     // -------------------------------------------------------------------------
//     if (DS2480B_Teensy::crc8(addr, 7) != addr[7])
//     {
//         Serial.println("  ERROR: ROM CRC mismatch — skipping device");
//         Serial.println();
//         return;
//     }

//     // -------------------------------------------------------------------------
//     // Confirm family code is DS18B20 (0x28)
//     // -------------------------------------------------------------------------
//     if (addr[0] != 0x28)
//     {
//         Serial.print("  WARNING: unexpected family code 0x");
//         Serial.print(addr[0], HEX);
//         Serial.println(" — expected 0x28 for DS18B20, continuing anyway");
//     }
//     else
//     {
//         Serial.println("  Chip   : DS18B20 confirmed");
//     }

//     // -------------------------------------------------------------------------
//     // Issue Convert T command to this specific sensor
//     // -------------------------------------------------------------------------
//     ds.reset();
//     ds.select(addr);
//     ds.write(0x44);     // Convert T — starts temperature measurement

//     // Wait for conversion.  DS18B20 at 12-bit resolution takes up to 750 ms.
//     // Polling the bus (reading 0 = busy, 1 = done) is cleaner than a fixed
//     // delay but requires parasitic power to be off; fixed delay is safer here.
//     delay(800);

//     // -------------------------------------------------------------------------
//     // Read scratchpad
//     // -------------------------------------------------------------------------
//     uint8_t present = ds.reset();
//     ds.select(addr);
//     ds.write(0xBE);     // Read Scratchpad

//     for (uint8_t i = 0; i < 9; i++)
//     {
//         data[i] = ds.read();
//     }

//     // -------------------------------------------------------------------------
//     // Validate scratchpad CRC (byte 8 must equal CRC of bytes 0–7)
//     // -------------------------------------------------------------------------
//     if (DS2480B_Teensy::crc8(data, 8) != data[8])
//     {
//         Serial.println("  ERROR: scratchpad CRC mismatch — discarding reading");
//         Serial.println();
//         return;
//     }

//     // Print raw scratchpad bytes for debugging
//     Serial.print("  Scratchpad (hex) : ");
//     for (uint8_t i = 0; i < 9; i++)
//     {
//         if (data[i] < 0x10) Serial.print("0");
//         Serial.print(data[i], HEX);
//         Serial.print(" ");
//     }
//     Serial.println();

//     // -------------------------------------------------------------------------
//     // Convert raw bytes to temperature
//     //
//     // The DS18B20 scratchpad bytes 0–1 hold a Q12.4 signed fixed-point value:
//     //   bits 15–4  integer part (two's complement degrees C)
//     //   bits  3–0  fractional part (1/16 °C per LSB)
//     //
//     // The config register (byte 4) bits 5–6 indicate resolution:
//     //   0x00 = 9-bit  (~93.75 ms),  mask low 3 bits
//     //   0x20 = 10-bit (~187.5 ms),  mask low 2 bits
//     //   0x40 = 11-bit (~375 ms),    mask low 1 bit
//     //   0x60 = 12-bit (~750 ms),    no masking needed
//     // -------------------------------------------------------------------------
//     int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);

//     uint8_t cfg = (data[4] & 0x60);
//     if      (cfg == 0x00) raw &= ~0x07;   //  9-bit
//     else if (cfg == 0x20) raw &= ~0x03;   // 10-bit
//     else if (cfg == 0x40) raw &= ~0x01;   // 11-bit
//     // 0x60 = 12-bit, no masking needed

//     float celsius    = static_cast<float>(raw) / 16.0f;
//     float fahrenheit = celsius * 1.8f + 32.0f;

//     Serial.print("  Resolution : ");
//     Serial.print(9 + (cfg >> 5));
//     Serial.println(" bit");

//     Serial.print("  Temperature: ");
//     Serial.print(celsius, 4);
//     Serial.print(" C  /  ");
//     Serial.print(fahrenheit, 4);
//     Serial.println(" F");

//     Serial.println();
// }




// RESET + Reserved Commands
#define CMD_RESET           0xC1
#define SET_DATA_MODE       0xE1
#define SET_CMD_MODE        0xE3
#define PULSE_TERMINATION   0xF1

// DS2480B_Detect config bytes
#define SET_PDSRC           0x17
#define SET_W1LD            0x45
#define SET_DSO_W0RT        0x5B
#define READ_RBR            0x0F
#define SEND_BIT            0x91

// Expected detect responses
#define RESP_PDSRC          0x16
#define RESP_W1LD           0x44
#define RESP_DSO_W0RT       0x5A
#define RESP_RBR            0x00
#define RESP_BIT            0x93

// Reset result codes (bits 1:0 of response)
#define RESET_SHORTED       0x00
#define RESET_PRESENCE      0x01
#define RESET_ALARM         0x02
#define RESET_NO_PRESENCE   0x03

// DS18B20 ROM Commands
#define SEARCH_ROM          0xF0
#define READ_ROM            0x33
#define MATCH_ROM           0x55
#define SKIP_ROM            0xCC
#define ALARM_SEARCH        0xEC

// DS18B20 Function Commands
#define CONVERT             0x44  // Initiates a single temperature conversion. Resulting data is stored in the 2-byte temperature register
#define WRITE_SCRATCHPAD    0x4E  // LSB. All three bytes MUST be written before the master issues a reset
#define READ_SCRATCHPAD     0xBE
#define COPY_SCRATCHPAD     0x48  // Copies ontents of bytes 2, 3 and 4 to EEPROM


// State
enum DS2480B_Mode
{   COMMAND_MODE,
    DATA_MODE
};
DS2480B_Mode currentMode = COMMAND_MODE;

// ----- HELPERS ----- //
void flushRXBuffer()
{
    while (Serial2.available()) Serial2.read();
}

// Read a single byte with timeout, returns -1 on timeout
int readByte(uint32_t timeoutMs = 500) // may need to change this
{
    uint32_t start = millis();
    while (Serial2.available() == 0)
    {
        if (millis() - start > timeoutMs)
            return -1;
    }
    return Serial2.read();
}

void ensureCommandMode()
{
    if (currentMode != COMMAND_MODE)
    {
        Serial2.write(SET_CMD_MODE);
        currentMode = COMMAND_MODE;
        delay(2);
        flushRXBuffer();
    }
}

void ensureDataMode()
{
    if (currentMode != DATA_MODE)
    {
        Serial2.write(SET_DATA_MODE);
        currentMode = DATA_MODE;
        delay(2); // no flush here because in data mode we are constantly getting responses.
    }
}


// ----- MAIN FUNCTIONS ----- //

/**
 *  Resets and configures the DS2480B with "flex mode" settings.
 *  @return true if chip responds correctly.
 */
bool DS2480B_Detect()
{
    // Send break via null byte at 4800 baud : "If break is not available on the host UART then
    // switching to a slower baud rate and sending a zero byte can simulate a break"
    Serial2.end();
    Serial2.begin(4800);
    Serial2.write((uint8_t)0x00);
    delay(2);


    Serial2.end();
    Serial2.begin(9600);
    delay(2);
    flushRXBuffer();

    Serial2.write(CMD_RESET);
    delay(2);
    flushRXBuffer();

    currentMode = COMMAND_MODE;

    // Send 5-byte flex config packet
    Serial2.write(SET_PDSRC);
    Serial2.write(SET_W1LD);
    Serial2.write(SET_DSO_W0RT);
    Serial2.write(READ_RBR);
    Serial2.write(SEND_BIT);

    // Read and validate 5-byte response
    uint8_t expected[5] = { RESP_PDSRC, RESP_W1LD, RESP_DSO_W0RT, RESP_RBR, RESP_BIT };
    for (int i = 0; i < 5; i++)
    {
        int b = readByte();
        if (b < 0)
        {
            Serial.print("DS2480B_Detect: timeout on byte ");
            Serial.println(i);
            return false;
        }
        if ((uint8_t)b != expected[i])
        {
            Serial.print("DS2480B_Detect: bad byte ");
            Serial.print(i);
            Serial.print(" got 0x");
            Serial.print((uint8_t)b, HEX);
            Serial.print(" expected 0x");
            Serial.println(expected[i], HEX);
            return false;
        }
    }

    Serial.println("DS2480B_Detect: success");
    return true;
}

/**
 *  Sends 1-Wire reset and checks for presence pulse.
 *  NOTE: we don't expect any pulse so likely this is a useless method??
 *  @return RESET_PRESENCE, RESET_ALARM, RESET_SHORTED, RESET_NO_PRESENCE, or -1 on failure.
 */
int OWReset()
{
    ensureCommandMode();
    flushRXBuffer();

    Serial2.write(0xC5);  // reset at standard/flex speed

    int b = readByte();
    if (b < 0)
    {
        Serial.println("OWReset: timeout");
        return -1;
    }

    uint8_t response = (uint8_t)b;
    uint8_t result = response & 0x03;  // bits 1:0 = presence result

    switch (result)
    {
        case RESET_SHORTED:
        {
            Serial.println("OWReset: 1-Wire shorted");
            break;
        }
        case RESET_PRESENCE:
        {
            Serial.println("OWReset: presence detected");
            break;
        }
        case RESET_ALARM:
        {
            Serial.println("OWReset: alarming presence");
            break;
        }
        case RESET_NO_PRESENCE:
        {
            Serial.println("OWReset: no presence");
            break;
        }
        default:
        {
            DS2480B_Detect();
            break;
        }
    }

    return result;
}


// ----- SINGLE BIT FUNCTIONS (UNLIKELY TO USE) ----- //

/**
 * Sends a single bit. Read is a write-1 with result sampled.
 * @return the bit read back (0 or 1), or -1 on failure.
 */
int OWWriteBit(uint8_t bit)
{
    ensureCommandMode();
    flushRXBuffer();

    // 100dss01: d=bit, ss=01 (flex speed)
    uint8_t cmd = 0x85 | ((bit & 0x01) << 4);
    Serial2.write(cmd);

    int b = readByte();
    if (b < 0)
    {
        Serial.println("OWWriteBit: timeout");
        return -1;
    }

    // bits 1:0 of response = result, 00=read 0, 11=read 1
    return ((uint8_t)b & 0x03) == 0x03 ? 1 : 0;
}

int OWReadBit()
{
    return OWWriteBit(1);  // read = write 1 and sample
}


// ----- LIKELY TO USE :) ----- //


/**
 *  Sends a single byte in Data Mode.
 *  Read is done by writing 0xFF and sampling the response.
 *  @return byte read back from 1-Wire, or -1 on failure.
 */
int OWWriteByte(uint8_t data)
{
    ensureDataMode();
    flushRXBuffer();

    // If data == 0xE3 it must be sent twice (AN192: duplicate E3s)
    Serial2.write(data);
    if (data == 0xE3) Serial2.write(data);

    int b = readByte();
    if (b < 0)
    {
        Serial.println("OWWriteByte: timeout");
        return -1;
    }
    return b;
}

int OWWriteBytes(uint8_t data)
{
    ensureDataMode();
    flushRXBuffer();

    // If data == 0xE3 it must be sent twice (AN192: duplicate E3s)
    Serial2.write(data);
    if (data == 0xE3) Serial2.write(data);

    int b = readByte();

    // If response length invalid, call DS2480B_Detect
    if (b < 0)
    {
        Serial.println("Length Error: Running DS2480B_Detect");
        if (!DS2480B_Detect())
        {
            Serial.println("OWWriteByte: DS2480B_Detect failed");
            return -1;
        }
    }

    return b;
}

int OWReadByte()
{
    return OWWriteByte(0xFF);  // read = write all 1s and sample
}


/**
 *  Same as above just multiple bytes at once.
 */
bool OWBlock(uint8_t* buf, uint8_t len)
{
    ensureDataMode();
    flushRXBuffer();

    for (int i = 0; i < len; i++)
    {
        Serial2.write(buf[i]);
        if (buf[i] == 0xE3) Serial2.write(buf[i]);  // duplicate E3s
    }

    for (int i = 0; i < len; i++)
    {
        int b = readByte();
        if (b < 0)
        {
            Serial.print("OWBlock: timeout on byte ");
            Serial.println(i);
            return false;
        }
        buf[i] = (uint8_t)b;
    }

    return true;
}

// uhh Table 5 copied :)
uint8_t  ROM_NO[8];
int      LastDiscrepancy       = 0;
int      LastFamilyDiscrepancy = 0;
bool     LastDeviceFlag        = false;
// id_bit_number ?
// search_direction ?

// On the first search we always go 0 at every discrepancy (take the left branch).
// This finds the first device. The last bit position where we had to choose is called LastDiscrepancy.
// On the next search we repeat the same path as before up to LastDiscrepancy, but this time go 1 at that position instead.
// This finds the next device.
// We keep doing this until there are no more discrepancies, meaning LastDiscrepancy = 0

// The 16 response bytes are 64 two-bit pairs
// Each pair is: [ ROM_ID bit | discrepancy flag ]
// odd bits = ROM ID    even bits = discrepancy flag

void buildSearchData(uint8_t* searchData)
{
    // Clear all 16 bytes
    memset(searchData, 0, 16);

    for (int id_bit_number = 1; id_bit_number <= 64; id_bit_number++)
    {
        int byteIndex = (id_bit_number - 1) / 4;
        int bitIndex  = ((id_bit_number - 1) % 4) * 2 + 1; // odd bits = search direction

        uint8_t search_direction = 0;

        if (id_bit_number < LastDiscrepancy)
        {
            // repeat same path as last time, use ROM_ID bit from previous search
            search_direction = (ROM_NO[(id_bit_number - 1) / 8] >> ((id_bit_number - 1) % 8)) & 0x01;
        }
        else if (id_bit_number == LastDiscrepancy)
        {
            // this is where we diverge, go 1 instead of 0
            search_direction = 1;
        }
        // else search_direction stays 0

        searchData[byteIndex] |= (search_direction << bitIndex); // how does this really work??
    }
}

bool parseSearchResponse(uint8_t* responseData)
{
    int newLastDiscrepancy       = 0;
    int newLastFamilyDiscrepancy = 0;

    for (int id_bit_number = 1; id_bit_number <= 64; id_bit_number++)
    {
        int byteIndex    = (id_bit_number - 1) / 4;
        int discrepenyBit   = ((id_bit_number - 1) % 4) * 2;   // even bits = discrepancy flag
        int directionBit = ((id_bit_number - 1) % 4) * 2 + 1;  // odd bits  = ROM ID bit

        uint8_t discrepancy = (responseData[byteIndex] >> discrepenyBit)   & 0x01;
        uint8_t romBit      = (responseData[byteIndex] >> directionBit) & 0x01;

        // Store ROM bit
        if (romBit)
        {
            ROM_NO[(id_bit_number - 1) / 8] |=  (1 << ((id_bit_number - 1) % 8));
        }
        else
        {
            ROM_NO[(id_bit_number - 1) / 8] &= ~(1 << ((id_bit_number - 1) % 8));
        }

        // Track discrepancies
        if (discrepancy && romBit == 0)
        {
            newLastDiscrepancy = id_bit_number;
            if (id_bit_number < 9)
                newLastFamilyDiscrepancy = id_bit_number;
        }
    }

    // Check CRC of ROM_NO
    uint8_t crc = 0;
    for (int i = 0; i < 8; i++)
    {
        uint8_t byte = ROM_NO[i];
        for (int j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }

    if (crc != 0)
    {
        Serial.println("parseSearchResponse: CRC invalid");
        LastDiscrepancy       = 0;
        LastFamilyDiscrepancy = 0;
        LastDeviceFlag        = false;
        return false;
    }

    // Update search state
    LastDiscrepancy       = newLastDiscrepancy;
    LastFamilyDiscrepancy = newLastFamilyDiscrepancy;

    if (LastDiscrepancy == 0)
    {
        LastDeviceFlag = true;
    }

    return true;
}


/**
 *  Finds the next device on the 1-Wire bus
 *  Call repeatedly to find all devices; resets when LastDeviceFlag is set.
 *  @return true if a device was found, false otherwise.
 */
bool OWSearch()
{
    // If last search found the last device, reset state
    if (LastDeviceFlag)
    {
        LastDiscrepancy       = 0;
        LastFamilyDiscrepancy = 0;
        LastDeviceFlag        = false;
        memset(ROM_NO, 0, 8);
    }

    // Reset, must find a device
    int resetResult = OWReset();
    if (resetResult != RESET_PRESENCE && resetResult != RESET_ALARM)
    {
        Serial.println("OWSearch: no devices found");
        return false;
    }

    // Build outbound search data
    uint8_t searchData[16];
    buildSearchData(searchData);

    // Send search packet
    Serial2.write(0xE1);        // Data mode
    Serial2.write(0xF0);        // Search ROM command
    Serial2.write(0xE3);        // Command mode
    Serial2.write(0xB5);        // Search Accelerator ON, ss=01 (1011 0101)
    Serial2.write(0xE1);        // Data mode

    for (int i = 0; i < 16; i++)
    {
        Serial2.write(searchData[i]);
        if (searchData[i] == 0xE3) Serial2.write(searchData[i]);  // duplicate E3s
    }

    Serial2.write(0xE3);        // Command mode
    Serial2.write(0xA5);        // Search Accelerator OFF, standard/flex speed


    //Read 17-byte response (echo of F0 + 16 data bytes)
    // First byte should be echo of search command F0
    int echo = readByte();
    if ((uint8_t)echo != 0xF0)
    {
        Serial.println("OWSearch: invalid echo byte, running DS2480B_Detect");
        DS2480B_Detect();
        return false;
    }

    // Read 16 response bytes
    uint8_t responseData[16];
    for (int i = 0; i < 16; i++)
    {
        int b = readByte();
        if (b < 0)
        {
            Serial.print("OWSearch: timeout on response byte ");
            Serial.println(i);
            DS2480B_Detect();
            return false;
        }
        responseData[i] = (uint8_t)b;
    }

    //Parse response and update search state
    if (!parseSearchResponse(responseData))
    {
        Serial.println("OWSearch: parse failed");
        return false;
    }

    // Print found ROM
    Serial.print("OWSearch: found device ROM = ");
    for (int i = 0; i < 8; i++)
    {
        if (ROM_NO[i] < 0x10) Serial.print('0');
        Serial.print(ROM_NO[i], HEX);
        if (i < 7) Serial.print(":");
    }
    Serial.println();

    return true;
}

void OWSearchAll()
{
    Serial.println("Searching for all 1-Wire devices...");
    int count = 0;

    while (OWSearch())
    {
        count++;
        Serial.print("Device ");
        Serial.print(count);
        Serial.print(": ");
        for (int i = 0; i < 8; i++)
        {
            if (ROM_NO[i] < 0x10) Serial.print('0');
            Serial.print(ROM_NO[i], HEX);
            if (i < 7) Serial.print(":");
        }
        Serial.println();

        if (LastDeviceFlag) break;
    }

    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" device(s)");
}

bool OWMatchROM(const uint8_t rom[8])
{
    if (OWReset() != RESET_PRESENCE)
        return false;

    OWWriteByte(MATCH_ROM);

    for (int i = 0; i < 8; i++)
        OWWriteByte(rom[i]);

    return true;
}

void ReadFirstFoundSensor()
{
    if (!OWSearch())
    {
        Serial.println("No sensor found");
        return;
    }

    uint8_t rom[8];
    memcpy(rom, ROM_NO, 8);

    Serial.print("Using ROM: ");
    for (int i = 0; i < 8; i++)
    {
        if (rom[i] < 0x10) Serial.print('0');
        Serial.print(rom[i], HEX);
        Serial.print(':');
    }
    Serial.println();

    // Start conversion
    if (!OWMatchROM(rom))
    {
        Serial.println("Match ROM failed");
        return;
    }

    OWWriteByte(CONVERT);

    delay(750);

    // Read scratchpad
    if (!OWMatchROM(rom))
    {
        Serial.println("Match ROM failed");
        return;
    }

    OWWriteByte(READ_SCRATCHPAD);

    uint8_t scratchpad[9];

    for (int i = 0; i < 9; i++)
    {
        int b = OWReadByte();

        if (b < 0)
        {
            Serial.println("Read failed");
            return;
        }

        scratchpad[i] = (uint8_t)b;
    }

    Serial.print("Scratchpad: ");

    for (int i = 0; i < 9; i++)
    {
        if (scratchpad[i] < 0x10) Serial.print('0');
        Serial.print(scratchpad[i], HEX);
        Serial.print(' ');
    }

    Serial.println();

    int16_t raw =
        ((int16_t)scratchpad[1] << 8) |
        scratchpad[0];

    float tempC = raw / 16.0f;

    Serial.print("Temperature = ");
    Serial.print(tempC);
    Serial.println(" C");
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600);
    delay(2500);

    while (true)
    {
        if (!DS2480B_Detect())
        {
            Serial.println("DS2480B Detect Failed");
            delay(1000);
            continue;
        }

        Serial.println("DS2480B OK");

        int result = OWReset();

        Serial.print("Reset Result = ");
        Serial.println(result);

        if (result == RESET_PRESENCE)
        {
            Serial.println("Device Found!");
            break;
        }

        delay(1000);
    }

    OWSearchAll();
}

void loop()
{
}