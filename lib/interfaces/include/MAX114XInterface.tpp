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

    : _channelTypes(channelTypes)
    , _spiPinCS(spiPinCS)
    , _spiPinSDI(spiPinSDI)
    , _spiPinSDO(spiPinSDO)
    , _spiPinCLK(spiPinCLK)
    , _spiSpeed(spiSpeed)
    , _currentChannel(0)
{   
    /* Constructs an AnalogChannel object for each channel of the ADC and sets scales and offsets*/
    for (int i = 0; i < MAX114X_ADC_NUM_CHANNELS; i++)
    {
        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);    
    }

    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);
    
    // Returns a compile time error if an incorrect version is given to the interface's template
    static_assert(
        (MAX114xVersion == 6 || MAX114xVersion == 7 || MAX114xVersion == 8 || MAX114xVersion == 9),
        "Invalid MAX114X version number"
    );

    // Page 15 of datasheet
    /* Assigns single mode selection codes to an array depending on version*/
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
uint16_t MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::getLastSampleRaw(int index) const
{
    return static_cast<uint16_t>(this->data.conversions[index].raw);
}
        
template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
float MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::getLastSampleConverted(int index) const
{
    return this->data.conversions[index].conversion;
}
        
template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
void MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_sample()
{
    byte command, b0, b1, b2;
    uint8_t selNum;

    // Resets loop after last channel is reached
    if (_currentChannel == MAX114X_ADC_NUM_CHANNELS)
    {
        _currentChannel = 0;
    }

    /* Creates variable corresponding to the current channels channelType (Single, Differential, Inverse Differential) using the array input by the user.
    * The index is divided by 2 since there is one channelType enum corresponding to each pair of channels 
    */
    CHANNEL_TYPE_e channelType = _channelTypes[_currentChannel / 2];

    switch (channelType) {
        case CHANNEL_TYPE_e::SINGLE:
        
            // The channel selection bits for single mode follows this array
            selNum = (_single_end_channel_to_select_map[_currentChannel]);
            break;

        case CHANNEL_TYPE_e::DIFFERENTIAL:
        
            // The channel selection bits for differential mode is the channel number halved and then truncated
            // The channelId is post-incremented so the sample() function does not send the same command byte for the other channel in the differential pair
            selNum = (_currentChannel / 2);
            break;
            
        case CHANNEL_TYPE_e::INV_DIFFERENTIAL:

            // The channel selection bits for inversed differential mode is the channel number halved, truncated, and with a 1 in the MSB
            // The channelId is post-incremented so the sample() function does not send the same command byte for the other channel in the differential pair
            selNum = ((_currentChannel / 2) | 0b100);
            break;
    }
    
    /* Page 14 of datasheet 
    * Create command byte to send to ADC
    */
    command =   (0x01 << 7) |                                                   // start bit
                ((selNum & 0x7) << 4) |                                         // channel number
                ((channelType == CHANNEL_TYPE_e::SINGLE ? 0x01 : 0x00) << 3) |  // single(1) or differential(0)
                (0x01 << 2) |                                                   // unipolar or !bipolar
                (0x01 << 1) |                                                   // external clock mode
                (0x01);                                                         // ^
    
    // initialize SPI bus. REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    digitalWrite(_spiPinCS, LOW);

    b0 = SPI.transfer(command);
    b1 = SPI.transfer(0x00); // dummy bytes to clock out data from the ADC
    b2 = SPI.transfer(0x00); // ^

    digitalWrite(_spiPinCS, HIGH);

    SPI.endTransaction();

    // First two bits of b1 are filler
    uint16_t value = ((b1 & 0x3F) << 8) | (b2 & 0xFF);
    
    /* Stores return bytes (14 bit ADC conversion) in lastSample member of analog channel class corresponding to the channel. FOR DIFFERENTIAL: data for the pair is stored in the lower of the two channels. Ex: 1 & 2 are a differential pair, the object for channel 1 holds the return value. */
    MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_channels[_currentChannel].lastSample = value;

    // Increments channel ID if the pair is differential or inverse differential
    if (channelType == CHANNEL_TYPE_e::DIFFERENTIAL || channelType == CHANNEL_TYPE_e::INV_DIFFERENTIAL)
    {
        _currentChannel++;
        MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>::_channels[_currentChannel].lastSample = value;
    }
    _currentChannel++;
}