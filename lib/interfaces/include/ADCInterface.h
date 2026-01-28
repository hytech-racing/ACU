#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"
#include "MAX114XInterface.h"

using pin = size_t;

namespace adc_default_parameters
{
    constexpr const float TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH = 0.2F;
    constexpr const float TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH = 3.0F;
    constexpr const float SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD = 12.0F;

    constexpr const uint32_t IMD_STARTUP_TIME = 2000;
    constexpr const float TEENSY41_MAX_INPUT_VOLTAGE = 3.3F;
}
struct ADCPinout_s 
{
    pin teensy_imd_ok_pin;
    pin teensy_precharge_pin;
    pin teensy_shdn_out_pin;
    pin teensy_ts_out_filtered_pin;
    pin teensy_pack_out_filtered_pin;
    pin teensy_bspd_current_pin;
    pin teensy_scaled_24V_pin;
};

struct ADCConversions_s
{
    float shutdown_conv_factor;
    float precharge_conv_factor;
    float pack_and_ts_out_conv_factor;
    float shdn_out_conv_factor;
    float bspd_current_conv_factor;
    float glv_conv_factor;
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
};

struct ADCChannels_s
{
    int iso_pack_diff_channel;
    int pack_voltage_sense_channel;
    int shunt_current_out_single_channel;
    int shunt_current_diff_channel;
    // int ts_out_filtered_channel;
    // int pack_out_filtered_channel;
};

struct ADCInterfaceParams_s
{
    ADCPinout_s pinout;
    ADCConversions_s conversions;
    ADCThresholds_s thresholds;
    ADCConfigs_s configs;
    ADCChannels_s channels;
    float bit_resolution;
};

class ADCInterface
{
public:
    ADCInterface(ADCPinout_s pinout,
                ADCConversions_s conversions,
                float bit_resolution,
                ADCChannels_s channels,
                // MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion>& MAX114XInterfaceInstance,
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
                    conversions.shutdown_conv_factor        = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.shutdown_conv_factor;
                    conversions.precharge_conv_factor       = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.precharge_conv_factor;
                    conversions.pack_and_ts_out_conv_factor = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.pack_and_ts_out_conv_factor;
                    conversions.shdn_out_conv_factor        = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.shdn_out_conv_factor;
                    conversions.bspd_current_conv_factor    = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.bspd_current_conv_factor;
                    conversions.glv_conv_factor             = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.glv_conv_factor;
                    return conversions;
                }(),
                thresholds, 
                configs, 
                channels,
                bit_resolution},
                // _max114x_instance(MAX114XInterfaceInstance)
                {}

    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
    */ 
    void init(uint32_t init_millis);

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

    // constexpr int ISO_PACK_N_CHANNEL         = 0;
    // constexpr int ISO_PACK_P_CHANNEL         = 1;
    // constexpr int PACK_VOLTAGE_SENSE_CHANNEL = 2;
    // constexpr int SHUNT_CURRENT_OUT_CHANNEL  = 3;
    // constexpr int SHUNT_CURRENT_P_CHANNEL    = 4;
    // constexpr int SHUNT_CURRENT_N_CHANNEL    = 5;
    // constexpr int TS_OUT_FILTERED_CHANNEL    = 6;
    // constexpr int PACK_OUT_FILTERED_CHANNEL  = 7;

    /**
     * @return ISO Pack Differential
    */
    int read_iso_pack();

    /**
     * @return pack voltage sense
    */
    int read_pack_voltage_sense();

    /**
     * @return shunt current single channel
    */
    int read_shunt_current();

    /**
     * @return shunt current differential channels
    */
    int read_differential_shunt_current();

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
    // MAX114XInterface<MAX114X_ADC_NUM_CHANNELS, MAX114xVersion> _max114x_instance;


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