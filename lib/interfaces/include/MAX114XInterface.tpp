/**
 * Code was based off of this data sheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1146-MAX1149.pdf
 * Page 14: Control byte format
 * Page 15: channel codes
 */


#include "MAX114XInterface.h"

#include <array>
#include <stdexcept>

template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::MAX114XInterface(
    const int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed,
    const float scales[MAX114X_ADC_NUM_CHANNELS], const float offsets[MAX114X_ADC_NUM_CHANNELS],
    const std::array<CHANNEL_TYPE_e, MAX114X_ADC_NUM_CHANNELS / 2>& channelTypes)

    : _spiPinCS(spiPinCS)
    , _spiPinSDI(spiPinSDI)
    , _spiPinSDO(spiPinSDO)
    , _spiPinCLK(spiPinCLK)
    , _spiSpeed(spiSpeed)
    , _channelTypes(channelTypes)
{
    for (int i = 0; i < MAX114X_ADC_NUM_CHANNELS; i++)
    {
        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);    
    }

    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);
    
    // Returns a compile time error if an incorrect version is given to the interface's template
    static_assert(
        (MAX114xVersion == 6 || MAX114xVersion == 7 ||
         MAX114xVersion == 8 || MAX114xVersion == 9),
        "Invalid MAX114X version number"
    );

    // Page 15 of datasheet
    switch (MAX114xVersion) {
        case 6:
        case 7:
            _single_end_channel_to_select_map = {0, 4, 1, 5};
            break;
        case 8:
        case 9:
            _single_end_channel_to_select_map = {0, 4, 1, 5, 2, 6, 3, 7};
            break;
        default:
            break;
    }
    
}

template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
void MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::tick()
{
    _sample();
    this->_convert();
}

template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
void MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_sample()
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

        // MOVE DEBUGGING TO TASKS
        Serial.print("b1: ");
        Serial.print(b1, HEX);
        Serial.print("  b2: ");
        Serial.print(b2, HEX);

        // uint16_t value = SPI.transfer16(command | channelIndex << 11);
        uint16_t value = ((b1 & 0x3F) << 8) | (b2 & 0xFF);
        Serial.print("  value: "); 
        Serial.println(value);
        // REMOVE ABOVE

        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_channels[channelIndex].lastSample = (value & 0x3FFF);
        
        // UPDATE CODE ABOVE

        digitalWrite(_spiPinCS, HIGH);
        delayMicroseconds(1); // MAX114XInterface Tcsh = 500ns
    }
    
    SPI.endTransaction();
}

template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
int MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_getSel(CHANNEL_TYPE_e channelType, int& channelId)
{
    switch (channelType) {
        case CHANNEL_TYPE_e::SINGLE:

            // The channel selection bits for single mode follows this array
            return _single_end_channel_to_select_map[channelId];

        case CHANNEL_TYPE_e::DIFFERENTIAL:

            // The channel selection bits for differential mode is the channel number halved and then truncated
            // The channelId is post-incremented so the sample() function does not send the same command byte for the other channel in the differential pair
            return channelId++ / 2;

        case CHANNEL_TYPE_e::INV_DIFFERENTIAL:

            // The channel selection bits for inversed differential mode is the channel number halved, truncated, and plus 4
            // The channelId is post-incremented so the sample() function does not send the same command byte for the other channel in the differential pair
            return (channelId++ / 2) + 4;
    }
}
