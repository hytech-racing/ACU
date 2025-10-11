#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

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
    float bit_resolution;
    float teensy41_max_input_voltage;
};

struct ADCInterfaceParams_s
{
    ADCPinout_s pinout;
    ADCConversions_s conversions;
    ADCThresholds_s thresholds;
    ADCConfigs_s configs;
};

class ADCInterface
{
public:
    ADCInterface(ADCPinout_s pinout,
                    ADCConversions_s conversions,
                    ADCThresholds_s thresholds,
                    ADCConfigs_s configs):
        _adc_params {
            {pinout},
            {conversions},
            {thresholds},
            {configs}
        }
        {};
    
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
    
private:
    const ADCInterfaceParams_s _adc_params = {};

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