#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"
#include "MAX114XInterface.h"

using pin = size_t;

namespace adc_default_parameters
{
    constexpr const float TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH = 0.5F;
    constexpr const float TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH = 2.8F;
    constexpr const float SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD = 12.0F;

    constexpr const uint32_t IMD_STARTUP_TIME = 2000;
    constexpr const float TEENSY41_MAX_INPUT_VOLTAGE = 3.3F;

    constexpr int MAX114X_VERSION = 8;
    constexpr size_t NUM_MAX1148_CHANNELS = 8;
};

struct ADCPinout_s 
{
    pin teensy_imd_ok_pin;
    pin teensy_precharge_pin;
    pin teensy_shdn_out_pin;

    pin teensy_hv_plus_out_ok_pin; 
    pin teensy_main_ok_pin;
    pin teensy_main_under_thresh_pin;
    pin teensy_precharge_under_thresh_pin; 

    pin teensy_ts_out_filtered_pin;
    pin teensy_pack_out_filtered_pin;
    pin teensy_bspd_current_pin;
    pin teensy_scaled_24V_pin;

    // MAX114X
    pin spiPinCS;
    pin spiPinSDI;
    pin spiPinSDO;
    pin spiPinCLK;
    
};

struct ADCConversions_s
{
    float shutdown_conv_factor;
    float precharge_conv_factor;
    float pack_and_ts_out_conv_factor;
    float shdn_out_conv_factor;
    float bspd_current_conv_factor;
    float glv_conv_factor;

    float std_5V_to_3V3_conversion_factor;
};

struct ADCThresholds_s
{
    float teensy41_min_digital_read_voltage_thresh;
    float teensy41_max_digital_read_voltage_thresh;
    float shutdown_voltage_digital_threshold;
};

struct ADCConfigs_s
{
    uint32_t imd_startup_time;
    float teensy41_max_input_voltage;
    int spiSpeed;
};

struct ADCChannels_s
{
    int iso_pack_n_channel;
    int iso_pack_p_channel;
    int pack_voltage_sense_channel;
    int shunt_current_out_channel;
    int shunt_current_p_channel;
    int shunt_current_n_channel;
    int ts_out_filtered_channel;
    int pack_out_filtered_channel;
};

struct ADCScales_s
{
    float iso_pack_n_scale;
    float iso_pack_p_scale;
    float pack_voltage_sense_scale;
    float shunt_current_out_scale;
    float shunt_current_p_scale;
    float shunt_current_n_scale;
    float ts_out_filtered_scale;
    float pack_out_filtered_scale;
};

struct ADCOffsets_s
{
    float iso_pack_n_offset;
    float iso_pack_p_offset;
    float pack_voltage_sense_offset;
    float shunt_current_out_offset;
    float shunt_current_p_offset;
    float shunt_current_n_offset;
    float ts_out_filtered_offset;
    float pack_out_filtered_offset;
};

struct MAX114XChannels_s
{
    CHANNEL_TYPE_e channelPair0;
    CHANNEL_TYPE_e channelPair1;
    CHANNEL_TYPE_e channelPair2;
    CHANNEL_TYPE_e channelPair3;
};

struct ADCInterfaceParams_s
{
    ADCPinout_s pinout;
    ADCConversions_s conversions;
    ADCThresholds_s thresholds;
    ADCConfigs_s configs;
    ADCChannels_s channels;
    ADCScales_s scales;
    ADCOffsets_s offsets;
    MAX114XChannels_s pairs;
    int spiSpeed;
    float bit_resolution;
};

class ADCInterface
{
public:
    ADCInterface(ADCPinout_s pinout,
                ADCConversions_s conversions,
                ADCChannels_s channels,
                ADCScales_s scales,
                ADCOffsets_s offsets,
                MAX114XChannels_s pairs,
                int spiSpeed,
                float bit_resolution,
                ADCThresholds_s thresholds = {
                    .teensy41_min_digital_read_voltage_thresh = adc_default_parameters::TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH,
                    .teensy41_max_digital_read_voltage_thresh = adc_default_parameters::TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH,
                    .shutdown_voltage_digital_threshold = adc_default_parameters::SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD
                },
                ADCConfigs_s configs = {
                    .imd_startup_time = adc_default_parameters::IMD_STARTUP_TIME,
                    .teensy41_max_input_voltage = adc_default_parameters::TEENSY41_MAX_INPUT_VOLTAGE
                }
        ): _adc_parameters { 
                pinout,
                [=]() mutable {
                    conversions.shutdown_conv_factor            = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.shutdown_conv_factor;
                    conversions.precharge_conv_factor           = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.precharge_conv_factor;
                    conversions.pack_and_ts_out_conv_factor     = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.pack_and_ts_out_conv_factor;
                    conversions.shdn_out_conv_factor            = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.shdn_out_conv_factor;
                    conversions.bspd_current_conv_factor        = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.bspd_current_conv_factor;
                    conversions.glv_conv_factor                 = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.glv_conv_factor;
                    conversions.std_5V_to_3V3_conversion_factor = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.std_5V_to_3V3_conversion_factor;
                    return conversions;
                }(),
                thresholds, 
                configs, 
                channels,
                scales,
                offsets,
                pairs,
                spiSpeed,
                bit_resolution},
            _max114x_instance(
                    _adc_parameters.pinout.spiPinCS,
                    _adc_parameters.pinout.spiPinSDI,
                    _adc_parameters.pinout.spiPinSDO,
                    _adc_parameters.pinout.spiPinCLK,
                    _adc_parameters.spiSpeed,
                    std::array<float, adc_default_parameters::NUM_MAX1148_CHANNELS> {
                        _adc_parameters.scales.iso_pack_n_scale,
                        _adc_parameters.scales.iso_pack_p_scale,
                        _adc_parameters.scales.pack_voltage_sense_scale,
                        _adc_parameters.scales.shunt_current_out_scale,
                        _adc_parameters.scales.shunt_current_p_scale,
                        _adc_parameters.scales.shunt_current_n_scale,
                        _adc_parameters.scales.ts_out_filtered_scale,
                        _adc_parameters.scales.pack_out_filtered_scale,
                    }.data(),
                    std::array<float, adc_default_parameters::NUM_MAX1148_CHANNELS> {
                        _adc_parameters.offsets.iso_pack_n_offset,
                        _adc_parameters.offsets.iso_pack_p_offset,
                        _adc_parameters.offsets.pack_voltage_sense_offset,
                        _adc_parameters.offsets.shunt_current_out_offset,
                        _adc_parameters.offsets.shunt_current_p_offset,
                        _adc_parameters.offsets.shunt_current_n_offset,
                        _adc_parameters.offsets.ts_out_filtered_offset,
                        _adc_parameters.offsets.pack_out_filtered_offset,
                    }.data(),
                    std::array<CHANNEL_TYPE_e, adc_default_parameters::NUM_MAX1148_CHANNELS / 2> {
                        _adc_parameters.pairs.channelPair0,
                        _adc_parameters.pairs.channelPair1,
                        _adc_parameters.pairs.channelPair2,
                        _adc_parameters.pairs.channelPair3
                    }
            )
            {}

    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
    */ 
    void init(uint32_t init_millis);

    /**
    * Samples from MAX114X adc
    */
    void tick();

    /**
     * @return the state of the IMD, HIGH = NO FAULT
    */
    bool read_imd_ok(uint32_t curr_millis);

    /**
     * @return the state of SHDN_OUT, HIGH = CAR was LATCHED
    */
    bool read_shdn_out();

    /**
     * @return the voltage of SHDN_OUT
    */
    volt read_shdn_voltage();

    /**
     * @return the state of PRECHARGE, HIGH = precharge relay is closed
     */
    bool read_precharge_out();

    /**
     * @return the voltage of PRECHARGE 
     */
    volt read_precharge_voltage();

    /**
     * @return HV+ out ok voltage -- output of AND gate that compares if the PACK_OUT voltage is within range
    */
    volt read_hv_plus_out_ok_voltage();

    /**
     * @return main ok voltage -- output of AND gate that checks MAIN_CHECK_OK and PACK_REF_UV_OK
    */
    volt read_main_ok_voltage();

    /**
     * @return main under threshold voltage -- dynamic voltage that is the output 0.9 Pack and TS_OUT hysteresis
    */
    volt read_main_under_threshold_voltage();

    /**
     * @return precharge under threshold voltage -- dynamic voltage that is output of 0.95 Pack and TS_OUT hysteresis
    */
    volt read_precharge_under_threshold_voltage();

    /**
     * @return voltage values of filtered TS OUT
    */
    volt read_ts_out_filtered();

    /**
     * @return voltage values of filtered PACK OUT
    */
    volt read_pack_out_filtered();

    /**
     * @return the current of BSPD
    */
    volt read_bspd_current();

    /**
     * @return voltage value of GLV, nominal 24V
    */
    volt read_global_lv_value();

    /**
     * @return shdn out voltage
    */
    volt read_shdn_out_voltage();

    /**
     * @return ISO Pack Differential
    */
    float read_iso_pack();

    /**
     * @return pack voltage sense
    */
    float read_pack_voltage_sense();

    /**
     * @return shunt current single channel
    */
    float read_shunt_current();

    /**
     * @return shunt current differential channels
    */
    float read_differential_shunt_current();

    /**
     * @return ADC parameters
     */
    const ADCInterfaceParams_s& get_adc_params() const;

    /**
     * @return true if still within IMD startup period
     */
    bool is_in_imd_startup_period() const;

private:
    const ADCInterfaceParams_s _adc_parameters = {};
    MAX114XInterface<adc_default_parameters::NUM_MAX1148_CHANNELS, adc_default_parameters::MAX114X_VERSION> _max114x_instance;

    /**
     * @brief true while within the IMD startup window (set in init())
     */
    bool _in_imd_startup_period;

    /**
     * @brief timestamp captured in init()
     */
    uint32_t _init_millis = 0;
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif