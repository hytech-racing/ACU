#ifndef DS2480B_H
#define DS2480B_H

#include <inttypes.h>

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
    #include "pins_arduino.h"
#endif

// ---------------------------------------------------------------------------
// Feature flags
// Define these to 0 before including this header to strip unused code.
// ---------------------------------------------------------------------------
#ifndef ONEWIRE_SEARCH
    #define ONEWIRE_SEARCH      1
#endif

#ifndef ONEWIRE_CRC
    #define ONEWIRE_CRC         1
#endif

#ifndef ONEWIRE_CRC8_TABLE
    #define ONEWIRE_CRC8_TABLE  1   // 1 = fast lookup table (~250 B flash),
#endif                              // 0 = compact bitwise algorithm

#ifndef ONEWIRE_CRC16
    #define ONEWIRE_CRC16       1
#endif

// ---------------------------------------------------------------------------
// SAM3X / Arduino Due compatibility shims
// ---------------------------------------------------------------------------
#if defined(__SAM3X8E__)
    #ifndef PROGMEM
        #define PROGMEM
    #endif
    #ifndef pgm_read_byte
        #define pgm_read_byte(addr) (*(const uint8_t *)(addr))
    #endif
#endif

// ---------------------------------------------------------------------------
// DS2480B protocol constants
// ---------------------------------------------------------------------------
#define DS2480B_DATA_MODE       0xE1
#define DS2480B_COMMAND_MODE    0xE3
#define DS2480B_PULSE_TERM      0xF1
#define DS2480B_RESET_CMD       0xC1
#define DS2480B_RESET_OK        0xCD
#define DS2480B_BAUD            9600

// Timeout for waiting on a reply byte from the DS2480B (milliseconds).
// Increase only if your bus is very slow / heavily loaded.
#ifndef DS2480B_REPLY_TIMEOUT_MS
    #define DS2480B_REPLY_TIMEOUT_MS 10
#endif

// ---------------------------------------------------------------------------
// DS2480B<SerialT>
//
// Templated on the serial port type so it works with any Arduino-compatible
// serial implementation without modification.
//
// SerialT contract (all standard Arduino serial types satisfy this):
//   void  begin(uint32_t baud)
//   void  write(uint8_t b)
//   int   read()
//   int   available()
//
// Teensy (recommended) usage:
//   DS2480B<HardwareSerial> bus(Serial1);
//
// Legacy AltSoftSerial (drop-in compat):
//   DS2480B<AltSoftSerial> bus(altPort);
// ---------------------------------------------------------------------------
template <typename SerialT>
class DS2480B
{
public:

    // -----------------------------------------------------------------------
    // Constructor
    // Stores a reference to the serial port — the port must outlive this
    // object.  Does not open the port; call begin() for that.
    // -----------------------------------------------------------------------
    explicit DS2480B(SerialT& port)
        : _port(port),
          _is_cmd_mode(false)
    {
#if ONEWIRE_SEARCH
        reset_search();
#endif
    }

    // -----------------------------------------------------------------------
    // begin
    // Opens the serial port at DS2480B_BAUD and sends the initialisation
    // byte.  Must be called once before any other method.
    // -----------------------------------------------------------------------
    void begin()
    {
        _port.begin(DS2480B_BAUD);
        delay(100);                         // DS2480B power-on / baud-detect settle
        _port.write(DS2480B_RESET_CMD);     // put DS2480B into a known state
        _is_cmd_mode = true;
    }

    // -----------------------------------------------------------------------
    // reset
    // Issues a 1-Wire reset pulse and listens for a presence pulse.
    // Returns 1 if at least one device responded, 0 otherwise.
    // -----------------------------------------------------------------------
    uint8_t reset()
    {
        _set_command_mode();
        _port.write(DS2480B_RESET_CMD);
        if (!_wait_for_reply()) return 0;
        return (_port.read() == DS2480B_RESET_OK) ? 1 : 0;
    }

    // -----------------------------------------------------------------------
    // beginTransaction / endTransaction
    // Convenience wrappers; match the naming used in ADCInterface.
    // -----------------------------------------------------------------------
    void beginTransaction()  { _set_data_mode(); }
    void endTransaction()    { _set_command_mode(); }

    // -----------------------------------------------------------------------
    // write_bit
    // Sends a single bit over 1-Wire.  Returns the bit echoed back.
    // -----------------------------------------------------------------------
    uint8_t write_bit(uint8_t v)
    {
        _set_command_mode();
        _port.write(v ? 0x91 : 0x81);
        if (!_wait_for_reply()) return 0;
        return _port.read() & 0x01;
    }

    // -----------------------------------------------------------------------
    // read_bit
    // Reads a single bit by writing a 1 and observing the bus response.
    // -----------------------------------------------------------------------
    uint8_t read_bit()
    {
        return write_bit(1);
    }

    // -----------------------------------------------------------------------
    // write
    // Sends one byte in data mode.  Escapes control bytes by doubling them.
    // The 'power' parameter is accepted for API compatibility but is not
    // needed with a DS2480B (it manages the bus pull-up internally).
    // -----------------------------------------------------------------------
    void write(uint8_t v, uint8_t power = 0)
    {
        (void)power;
        _set_data_mode();
        _port.write(v);

        // Escape any byte that matches a DS2480B command-mode control byte
        if (v == DS2480B_DATA_MODE    ||
            v == DS2480B_COMMAND_MODE ||
            v == DS2480B_PULSE_TERM)
        {
            _port.write(v);
        }

        if (!_wait_for_reply()) return;
        (void)_port.read();     // discard echo
    }

    // -----------------------------------------------------------------------
    // writeCmd
    // Sends one byte in command mode and discards the response byte.
    // -----------------------------------------------------------------------
    void writeCmd(uint8_t v, uint8_t power = 0)
    {
        (void)power;
        _set_command_mode();
        _port.write(v);
        if (!_wait_for_reply()) return;
        (void)_port.read();
    }

    // -----------------------------------------------------------------------
    // write_bytes
    // -----------------------------------------------------------------------
    void write_bytes(const uint8_t* buf, uint16_t count, bool power = false)
    {
        for (uint16_t i = 0; i < count; i++) write(buf[i]);
    }

    // -----------------------------------------------------------------------
    // read
    // Clocks out one byte from the 1-Wire bus by sending 0xFF in data mode.
    // -----------------------------------------------------------------------
    uint8_t read()
    {
        _set_data_mode();
        _port.write(0xFF);
        if (!_wait_for_reply()) return 0;
        return static_cast<uint8_t>(_port.read());
    }

    // -----------------------------------------------------------------------
    // read_bytes
    // -----------------------------------------------------------------------
    void read_bytes(uint8_t* buf, uint16_t count)
    {
        for (uint16_t i = 0; i < count; i++) buf[i] = read();
    }

    // -----------------------------------------------------------------------
    // select
    // Issues a Match ROM command followed by the 8-byte ROM address, 
    // targeting a specific device on the bus.
    // -----------------------------------------------------------------------
    void select(const uint8_t rom[8])
    {
        write(0x55);                            // Match ROM
        for (uint8_t i = 0; i < 8; i++) write(rom[i]);
    }

    // -----------------------------------------------------------------------
    // skip
    // Issues a Skip ROM command, addressing all devices simultaneously.
    // -----------------------------------------------------------------------
    void skip()
    {
        write(0xCC);                            // Skip ROM
    }

    // Accepted for API compatibility; no action needed for DS2480B.
    void depower() {}

    // -----------------------------------------------------------------------
    // Search ROM algorithm
    // -----------------------------------------------------------------------
#if ONEWIRE_SEARCH

    // Reset search state so the next call to search() starts from scratch.
    void reset_search()
    {
        _last_discrepancy        = 0;
        _last_device_flag        = false;
        _last_family_discrepancy = 0;
        for (uint8_t i = 0; i < 8; i++) _rom_no[i] = 0;
    }

    // Seed the search to find only devices with the given family code first.
    void target_search(uint8_t family_code)
    {
        _rom_no[0] = family_code;
        for (uint8_t i = 1; i < 8; i++) _rom_no[i] = 0;
        _last_discrepancy        = 64;
        _last_family_discrepancy = 0;
        _last_device_flag        = false;
    }

    // Enumerate the next device on the bus.
    // Returns 1 and writes the ROM address into newAddr on success.
    // Returns 0 when no more devices are found or a bus error occurs.
    uint8_t search(uint8_t* newAddr)
    {
        uint8_t       id_bit_number    = 1;
        uint8_t       last_zero        = 0;
        uint8_t       rom_byte_number  = 0;
        uint8_t       rom_byte_mask    = 1;
        uint8_t       search_direction = 0;
        uint8_t       search_result    = 0;
        uint8_t       id_bit, cmp_id_bit;

        if (_last_device_flag)
        {
            // Already found the last device; reset for next round
            reset_search();
            return 0;
        }

        if (!reset())
        {
            reset_search();
            return 0;
        }

        write(0xF0);    // Search ROM command

        do
        {
            id_bit     = read_bit();
            cmp_id_bit = read_bit();

            // No devices present
            if (id_bit == 1 && cmp_id_bit == 1) break;

            if (id_bit != cmp_id_bit)
            {
                search_direction = id_bit;
            }
            else
            {
                if (id_bit_number < _last_discrepancy)
                    search_direction = ((_rom_no[rom_byte_number] & rom_byte_mask) > 0);
                else
                    search_direction = (id_bit_number == _last_discrepancy);

                if (search_direction == 0)
                {
                    last_zero = id_bit_number;
                    if (last_zero < 9) _last_family_discrepancy = last_zero;
                }
            }

            if (search_direction == 1)
                _rom_no[rom_byte_number] |=  rom_byte_mask;
            else
                _rom_no[rom_byte_number] &= ~rom_byte_mask;

            write_bit(search_direction);

            id_bit_number++;
            rom_byte_mask <<= 1;

            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
        }
        while (rom_byte_number < 8);

        if (id_bit_number >= 65)
        {
            _last_discrepancy = last_zero;
            if (_last_discrepancy == 0) _last_device_flag = true;
            search_result = 1;
        }

        if (!search_result || !_rom_no[0])
        {
            reset_search();
            return 0;
        }

        for (uint8_t i = 0; i < 8; i++) newAddr[i] = _rom_no[i];
        return search_result;
    }

#endif // ONEWIRE_SEARCH

    // -----------------------------------------------------------------------
    // CRC utilities (static — no bus state required)
    // -----------------------------------------------------------------------
#if ONEWIRE_CRC

    static uint8_t crc8(const uint8_t* addr, uint8_t len)
    {
#if ONEWIRE_CRC8_TABLE
        // Lookup table from Dallas Semiconductor application note 27.
        // Stored in flash (PROGMEM on AVR; plain const elsewhere).
        static const uint8_t PROGMEM dscrc_table[] = {
              0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
            157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
             35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
            190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
             70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
            219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
            101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
            248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
            140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
             17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
            175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
             50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
            202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
             87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
            233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
            116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
        };

        uint8_t crc = 0;
        while (len--) crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
        return crc;
#else
        uint8_t crc = 0;
        while (len--)
        {
            uint8_t inbyte = *addr++;
            for (uint8_t i = 8; i; i--)
            {
                uint8_t mix = (crc ^ inbyte) & 0x01;
                crc >>= 1;
                if (mix) crc ^= 0x8C;
                inbyte >>= 1;
            }
        }
        return crc;
#endif // ONEWIRE_CRC8_TABLE
    }

#if ONEWIRE_CRC16

    static bool check_crc16(const uint8_t* input,
                             uint16_t       len,
                             const uint8_t* inverted_crc,
                             uint16_t       crc = 0)
    {
        crc = ~crc16(input, len, crc);
        return ((crc & 0xFF) == inverted_crc[0]) && ((crc >> 8) == inverted_crc[1]);
    }

    static uint16_t crc16(const uint8_t* input,
                           uint16_t       len,
                           uint16_t       crc = 0)
    {
        static const uint8_t oddparity[16] =
            { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

        for (uint16_t i = 0; i < len; i++)
        {
            uint16_t cdata = input[i];
            cdata = (cdata ^ crc) & 0xFF;
            crc >>= 8;
            if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4]) crc ^= 0xC001;
            cdata <<= 6; crc ^= cdata;
            cdata <<= 1; crc ^= cdata;
        }
        return crc;
    }

#endif // ONEWIRE_CRC16
#endif // ONEWIRE_CRC

private:

    SerialT& _port;
    bool     _is_cmd_mode;

#if ONEWIRE_SEARCH
    uint8_t _rom_no[8];
    uint8_t _last_discrepancy;
    uint8_t _last_family_discrepancy;
    bool    _last_device_flag;
#endif

    // -----------------------------------------------------------------------
    // _set_data_mode / _set_command_mode
    // Only write the mode-switch byte when the mode actually changes to
    // avoid unnecessary bus traffic.
    // -----------------------------------------------------------------------
    void _set_data_mode()
    {
        if (_is_cmd_mode)
        {
            _port.write(DS2480B_DATA_MODE);
            _is_cmd_mode = false;
        }
    }

    void _set_command_mode()
    {
        if (!_is_cmd_mode)
        {
            _port.write(DS2480B_COMMAND_MODE);
            _is_cmd_mode = true;
        }
    }

    // -----------------------------------------------------------------------
    // _wait_for_reply
    // Blocks until a byte is available on the serial port or the timeout
    // expires.  Uses millis() so it is safe on any clock speed.
    // Returns true if data is available, false on timeout.
    // -----------------------------------------------------------------------
    bool _wait_for_reply()
    {
        uint32_t start = millis();
        while (!_port.available())
        {
            if ((millis() - start) >= DS2480B_REPLY_TIMEOUT_MS) return false;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// Convenience alias for Teensy hardware serial ports (Serial1 … Serial7).
// Use this type throughout the project instead of spelling out the template.
// ---------------------------------------------------------------------------
using DS2480B_Teensy = DS2480B<HardwareSerial>;

#endif // DS2480B_H