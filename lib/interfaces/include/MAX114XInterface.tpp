#include "MAX114XInterface.h"
#include <array>

template <int MAX1148X_ADC_NUM_CHANNELS>
MAX114XInterface<MAX1148X_ADC_NUM_CHANNELS>::MAX114XInterface(const int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed, const float scales[MAX1148X_ADC_NUM_CHANNELS], const float offsets[MAX1148X_ADC_NUM_CHANNELS])
: _spiPinCS(spiPinCS)
, _spiPinSDI(spiPinSDI)
, _spiPinSDO(spiPinSDO)
, _spiPinCLK(spiPinCLK)
, _spiSpeed(spiSpeed)
{
    for (int i = 0; i < MAX1148X_ADC_NUM_CHANNELS; i++)
    {
        MAX114XInterface<MAX1148X_ADC_NUM_CHANNELS>::_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);
}

template <int MAX1148X_ADC_NUM_CHANNELS>
void MAX114XInterface<MAX1148X_ADC_NUM_CHANNELS>::tick()
{
    _sample();
    this->_convert();
}

template <int MAX1148X_ADC_NUM_CHANNELS>
void MAX114XInterface<MAX1148X_ADC_NUM_CHANNELS>::_sample()
{
    // uint16_t command = (
    //     (0b1 << 15) |    // start bit
    //     (0b1 << 14)      // single ended mode
    // );
    byte command, b0, b1, b2;

    // initialize SPI bus. REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    for (int channelIndex = 0; channelIndex < MAX1148X_ADC_NUM_CHANNELS; channelIndex++)
    {
        digitalWrite(_spiPinCS, LOW);
        
        // UPDATE CODE BELOW
        
        command = ((0x01 << 7) |                     // start bit
                    ((channelIndex & 0x07) << 4) |   // channel number
                    (0x01 << 3) |                    // single or differential
                    (0x01 << 2) |                    // unipolar or !bipolar
                    (0x01 << 1) |                    // external clock mode
                    (0x01 << 0));                    // 
        b0 = SPI.transfer(command);
        b1 = SPI.transfer(0x00);
        b2 = SPI.transfer(0x00);

        Serial.print("b0: ");
        Serial.println(b0, HEX);
        Serial.print("b1: ");
        Serial.println(b1, HEX);
        Serial.print("b2: ");
        Serial.println(b2, HEX);

        // uint16_t value = SPI.transfer16(command | channelIndex << 11);
        uint16_t value = (b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5;
        MAX114XInterface<MAX1148X_ADC_NUM_CHANNELS>::_channels[channelIndex].lastSample = (value & 0x3FFF);
        
        // UPDATE CODE ABOVE

        digitalWrite(_spiPinCS, HIGH);
        delayMicroseconds(1); // MAX114XInterface Tcsh = 500ns
    }
    
    SPI.endTransaction();
}