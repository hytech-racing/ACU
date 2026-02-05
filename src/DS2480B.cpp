/*
Modified DS2480B library to support HardwareSerial and other Stream-based serial interfaces.
Based on original OneWire library and DS2480B adaptations.
*/

#include "DS2480B.h"

DS2480B::DS2480B(Stream &port)
{
	_port = &port;  // Store pointer to the Stream object
// #if ONEWIRE_SEARCH
	// reset_search();
// #endif
}

void DS2480B::begin()
{
	_port->write(0xC1);
	isCmdMode = true;
	Serial.println("Begin successful");
}

uint8_t DS2480B::reset(void)
{
	uint8_t r;
	Serial.println("Reset Called!");

	while(_port->available()) _port->read();

	Serial.println("Reset Flushed!");

	commandMode();

	_port->write(0xC1); //0xC1 is the set reset command

	Serial.println("Command Sent!");

	while (!_port->available()); //waits for tx pulse



	r = _port->read();

	//Debug output logic
	if (r == 0xCD || r == 0xED){
		Serial.println("Reset Successful!");
	} else {
		Serial.println("Reset Failed!");
	}

	return r;
}

void DS2480B::dataMode()
{
	if (isCmdMode)
	{
		_port->write(DATA_MODE);
		isCmdMode = false;
	}
}

void DS2480B::commandMode()
{
	if (!isCmdMode)
	{
		_port->write(COMMAND_MODE);
		isCmdMode = true;
	}
}

void DS2480B::beginTransaction()
{
	dataMode();
}

void DS2480B::endTransaction()
{
	commandMode();
}

bool DS2480B::waitForReply()
{
	for (uint16_t i = 0; i < 30000; i++)
	{
		if (_port->available()) return true;
	}
	return false;
}

uint8_t DS2480B::write_bit(uint8_t v)
{
	uint8_t val;
	commandMode();
	if (v == 1) _port->write(0x91);
	else _port->write(0x81);
	if (!waitForReply()) return 0;
	val = _port->read();
	return val & 1;
}

uint8_t DS2480B::read_bit(void)
{
	uint8_t r = write_bit(1);
	return r;
}

void DS2480B::write(uint8_t v, uint8_t power) {
	uint8_t r;

	dataMode();

	_port->write(v);
	if (v == DATA_MODE || v == COMMAND_MODE || v == PULSE_TERM) _port->write(v);
	if (!waitForReply()) return;
	r = _port->read();
}

void DS2480B::writeCmd(uint8_t v, uint8_t power)
{
	uint8_t r;

	commandMode();

	_port->write(v);

	if (!waitForReply()) return;
	r = _port->read();
}

void DS2480B::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
	for (uint16_t i = 0 ; i < count ; i++) write(buf[i]);
}

uint8_t DS2480B::read() {
	uint8_t r;

	dataMode();

	_port->write(0xFF);
	if (!waitForReply()) return 0;
	r = _port->read();
	return r;
}

void DS2480B::read_bytes(uint8_t *buf, uint16_t count) {
	for (uint16_t i = 0 ; i < count ; i++)
		buf[i] = read();
}

void DS2480B::select(const uint8_t rom[8])
{
	uint8_t i;

	dataMode();
	write(0x55);

	for (i = 0; i < 8; i++) write(rom[i]);
}

void DS2480B::skip()
{
	dataMode();
	write(0xCC);
}

void DS2480B::depower()
{

}

#if ONEWIRE_SEARCH

//Removing Search Algorithm

/* void DS2480B::reset_search() 
{
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	for(int i = 7; ; i--) {
		ROM_NO[i] = 0;
		if ( i == 0) break;
	}
}

void DS2480B::target_search(uint8_t family_code)
{
	ROM_NO[0] = family_code;
	for (uint8_t i = 1; i < 8; i++)
		ROM_NO[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	LastDeviceFlag = FALSE;
}

uint8_t DS2480B::search(uint8_t *newAddr)
{
	uint8_t ROM[16];

	if(reset()){
		Serial.println("Search Called, Reset Performed!");
		_port->write(0xE1); //Set Data Mode
		_port->write(0xF0); //Search ROM Command
		_port->write(0xE3); //Set Command Mode
		_port->write(0xB1); //Search Accelerator On
		_port->write(0xE1); //Set Data Mode
		Serial.println("Search Accelerator On!");
	} else {
		Serial.println("Search Failed!");
		return;
	}

	for(int i = 0; i < 16; i++){ //First, send out empty 16 bytes 
		_port->write((uint8_t)0x00);
	}

	for(int i = 0; i < 16; i++){ //Second, read the returned bytes
		ROM[i] = _port->read();
	}

	for(int i = 0; i < 16; i++){ //Debug Step 1
		Serial.println(ROM[i]);
	}

	//return;
	return ROM[16];
}
	if (!search_result || !ROM_NO[0])
	{
		Serial.println("No Device Found");
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = FALSE;
	} 
	for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
	return search_result; 
} */ 


#endif

#if ONEWIRE_CRC

#if ONEWIRE_CRC8_TABLE

static const uint8_t PROGMEM dscrc_table[] = {
	0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
	35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
	190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
	70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
	219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
	101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
	140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
	17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
	50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
	202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
	87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

uint8_t DS2480B::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
	}
	return crc;
}
#else

uint8_t DS2480B::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}
#endif

#if ONEWIRE_CRC16
bool DS2480B::check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
	crc = ~crc16(input, len, crc);
	return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t DS2480B::crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
	static const uint8_t oddparity[16] =
		{ 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

	for (uint16_t i = 0 ; i < len ; i++) {
		uint16_t cdata = input[i];
		cdata = (cdata ^ crc) & 0xff;
		crc >>= 8;

		if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
			crc ^= 0xC001;

		cdata <<= 6;
		crc ^= cdata;
		cdata <<= 1;
		crc ^= cdata;
	}
	return crc;
}
#endif

#endif