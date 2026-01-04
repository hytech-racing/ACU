#ifndef DS2480B_h
#define DS2480B_h

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#if ONEWIRE_SEARCH
#endif

#define FALSE 0
#define TRUE  1

#define DATA_MODE		0xE1
#define COMMAND_MODE	0xE3
#define PULSE_TERM		0xF1

// Feature configuration
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE 1
#endif

#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif

class DS2480B
{
  private:
	Stream* _port;  // Use Stream pointer for flexibility (HardwareSerial, SoftwareSerial, etc.)
	bool isCmdMode;

#if ONEWIRE_SEARCH
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    uint8_t LastDeviceFlag;
#endif

    bool waitForReply();

  public:
    DS2480B(Stream& port);  // Accept any Stream-derived class

	void begin();

    uint8_t reset(void);

	void beginTransaction();
	void endTransaction();

	void commandMode();
	void dataMode();

    void select(const uint8_t rom[8]);
    void skip(void);

    void write(uint8_t v, uint8_t power = 0);
	void writeCmd(uint8_t v, uint8_t power = 0);
    void write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);

    uint8_t read(void);
    void read_bytes(uint8_t *buf, uint16_t count);

    uint8_t write_bit(uint8_t v);
    uint8_t read_bit(void);

    void depower(void);

#if ONEWIRE_SEARCH
    void reset_search();
    void target_search(uint8_t family_code);
    uint8_t search(uint8_t *newAddr);
#endif

#if ONEWIRE_CRC
    static uint8_t crc8(const uint8_t *addr, uint8_t len);

#if ONEWIRE_CRC16
    static bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc = 0);
    static uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc = 0);
#endif
#endif
};

#endif