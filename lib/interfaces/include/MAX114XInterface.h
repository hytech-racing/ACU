#ifndef MAX114XINTERFACE_H
#define MAX114XINTERFACE_H

#include "AnalogSensorsInterface.h"
#include <SPI.h>
#include <Arduino.h>

/**
 * Enum representing the different channel configurations in MAX114X ADCs (SINGLE, DIFFERENTIAL, or INV_DIFFERENTIAL)
 */
enum class CHANNEL_TYPE_e{
    SINGLE, ///< single channel
    DIFFERENTIAL, ///< +- differential pair
    INV_DIFFERENTIAL, ///< -+ differential pair
};

/**
 * The MAX114X_ADC is a concrete subclass of the AnalogMultiSensor parent class. This allows
 * for SPI communication with any version of the MAX114X_ADC, which can be a 4-channel or 8-channel sensor.
 * IMPORTANT - must call SPI.begin() once and only once before instantiating any MCP_ADC object!!
 * @param MAX114X_ADC_NUM_CHANNELS number of channels for the ADC
 * @param MAX114xVersion 6, 7, 8, or 9, corresponding to the MAX1146, MAX1147, MAX1148, and MAX1149 ADCs
 */
template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
class MAX114XInterface : public AnalogMultiSensor<MAX114X_ADC_NUM_CHANNELS>
{
public:
    /* Constructors */
    /**
     * Constructs a MAX114X ADC interface of the specified ADC model, number of channels, and channel types.
     * @param channelTypes Constructs an array of channel types (Single, Differential, Inverse Differential) that is half the length of the ADC's channels. This is because each element of the array corresponds to a pair of channels.
     */
    MAX114XInterface(int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int adc_not_shdn_pin, const int spiSpeed, const float scales[MAX114X_ADC_NUM_CHANNELS], const float offsets[MAX114X_ADC_NUM_CHANNELS], const std::array<CHANNEL_TYPE_e, MAX114X_ADC_NUM_CHANNELS / 2>& channelTypes);

    /* Functions */
    /**
     * initialize pins particularly CS and ADC_!SHDN
    */
    void init();

    /**
     * Calls sample() and convert(). After calling tick(), this MCP_ADC's data can be accessed using the get() command.
     */
    void tick() override;
    
    /**
     * Gets raw 14 bit value of a channel for a sample
     */
    uint16_t get_last_sample_raw(int index) const;
    
    /**
     * Gets real value (current/voltage) of a channel for a sample
     */
    float get_last_sample_converted(int index) const;
    
private:

    /**
     * Samples the MCP_ADC over SPI. Samples all eight channels and, in accordance with the AnalogMultiSensor's function
     * contract, stores the raw sampled values into each AnalogChannel's lastSample instance variable.
     */
    void _sample() override;

    /**
     * Channel configuration is defined per channel pair (two physical channels).
     * This array stores the channel type for each pair of channels in the ADC.
     * Each pair may be configured as SINGLE, DIFFERENTIAL, or INV_DIFFERENTIAL as defined in the enclosed enum.
     * SINGLE indicates both channels in the pair are single-ended inputs and operate separately.
     */
    const std::array<CHANNEL_TYPE_e, MAX114X_ADC_NUM_CHANNELS / 2> _channelTypes;
    
    const int _spiPinCS;
    const int _spiPinSDI;
    const int _spiPinSDO;
    const int _spiPinCLK;
    const int _adc_not_shdn_pin;
    const int _spiSpeed;
    int _currentChannel;
    
    /**
     * The select bits for single-ended channels are all over the place and do not follow a logical mapping.
     * This array stores the specific single-ended select-bit mapping for each channel as defined in the datasheet. 
     */
    std::array<uint8_t, MAX114X_ADC_NUM_CHANNELS> _single_end_channel_to_select_map;
};

template <int MAX114X_ADC_NUM_CHANNELS, int MAX114xVersion>
using MAX114XInterfaceInstance = etl::singleton<MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>>;

#include "MAX114XInterface.tpp"

#endif /* MAX114X_INTERFACE */
