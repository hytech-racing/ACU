/* Code was based off of this data sheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1146-MAX1149.pdf
Page 14: Control byte format
Page 15: channel codes
*/

#include "MAX114XInterface.h"
#include <array>
#include <stdexcept>

template <int MAX114X_ADC_NUM_CHANNELS>
MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::MAX114XInterface(
    const int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed,
    const float scales[MAX114X_ADC_NUM_CHANNELS], const float offsets[MAX114X_ADC_NUM_CHANNELS],
    // const std::array<CHANNEL_TYPE_e, MAX114X_ADC_NUM_CHANNELS / 2>& channelTypes,
    const CHANNEL_TYPE_e channelTypes[MAX114X_ADC_NUM_CHANNELS],
    const int max114xVersion)

    : _spiPinCS(spiPinCS)
    , _spiPinSDI(spiPinSDI)
    , _spiPinSDO(spiPinSDO)
    , _spiPinCLK(spiPinCLK)
    , _spiSpeed(spiSpeed)
    , _channelTypes(channelTypes)
{
    for (int i = 0; i < MAX114X_ADC_NUM_CHANNELS; i++)
    {
        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);    
    }

    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);
    
    switch (max114xVersion) {
    case 6:
    case 7:
        _single_end_channel_to_select_map = {0, 4, 1, 5};
        break;
    case 8:
    case 9:
        _single_end_channel_to_select_map = {0, 4, 1, 5, 2, 6, 3, 7};
        break;
    default:
        // THROW ERROR
        break;
    }
    
}

template <int MAX114X_ADC_NUM_CHANNELS>
void MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::tick()
{
    _sample();
    this->_convert();
}

template <int MAX114X_ADC_NUM_CHANNELS>
void MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::_sample()
{
    // uint16_t command = (
    //     (0b1 << 15) |    // start bit
    //     (0b1 << 14)      // single ended mode
    // );
    byte command, b0, b1, b2;

    // initialize SPI bus. REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    for (int channelIndex = 0; channelIndex < MAX114X_ADC_NUM_CHANNELS; channelIndex++)
    {
        digitalWrite(_spiPinCS, LOW);
        
        CHANNEL_TYPE_e channelType = _channelTypes[channelIndex / 2];
        // UPDATE CODE BELOW

        command = ((0x01 << 7) |                                                   // start bit
                   ((_getSel(channelType, channelIndex) & 0x07) << 4) |            // channel number
                   ((channelType == CHANNEL_TYPE_e::SINGLE ? 0x01 : 0x00) << 3) |  // single or differential
                   (0x01 << 2) |                                                   // unipolar or !bipolar
                   (0x01 << 1) |                                                   // external clock mode
                   (0x01 << 1));                                                   // 
        b0 = SPI.transfer(command);
        b1 = SPI.transfer(0x00);
        b2 = SPI.transfer(0x00);

        Serial.print("b1: ");
        Serial.print(b1, HEX);
        Serial.print("  b2: ");
        Serial.print(b2, HEX);

        // uint16_t value = SPI.transfer16(command | channelIndex << 11);
        uint16_t value = ((b1 & 0x3F) << 8) | (b2 & 0xFF);
        Serial.print("  value: ");
        Serial.println(value);
        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::_channels[channelIndex].lastSample = (value & 0x3FFF);
        
        // UPDATE CODE ABOVE

        digitalWrite(_spiPinCS, HIGH);
        delayMicroseconds(1); // MAX114XInterface Tcsh = 500ns
    }
    
    SPI.endTransaction();
}

template <int MAX114X_ADC_NUM_CHANNELS>
int MAX114XInterface<MAX114X_ADC_NUM_CHANNELS>::_getSel(CHANNEL_TYPE_e channelType, int& channelId)
{
    switch (channelType) {
        case CHANNEL_TYPE_e::SINGLE:
            return _single_end_channel_to_select_map[channelId];
        case CHANNEL_TYPE_e::DIFFERENTIAL:
            return channelId++ / 2;
        case CHANNEL_TYPE_e::INV_DIFFERENTIAL:
            return (channelId++ / 2) + 4;
    }
}
