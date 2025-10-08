#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

namespace pin_default_params
{
    constexpr const pin TS_OUT_FILTERED_PIN = 17;
    constexpr const pin PACK_OUT_FILTERED_PIN = 18;
    constexpr const pin IMD_OK_PIN = 25; // < READ from IMD hardware, go to FAULT state if HIGH
    constexpr const pin PRECHARGE_PIN = 26; // READ from PRECHARGE
    constexpr const pin BSPD_CURRENT_PIN = 27;
    constexpr const pin SHDN_OUT_PIN = 38; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    constexpr const pin SCALED_24V_PIN = 39;
}

namespace conv_default_params
{
    constexpr const float SHUTDOWN_CONV_FACTOR = 0.1155F; // voltage divider -> 4.7k / (4.7k + 36k)
    constexpr const float PRECHARGE_CONV_FACTOR = 0.6623F; // voltage divider -> 10k / (5.1k + 10k)
    constexpr const float PACK_AND_TS_OUT_CONV_FACTOR = 0.00482F;
    constexpr const float SHDN_OUT_CONV_FACTOR = 0.1036F;
    constexpr const float BSPD_CURRENT_CONV_FACTOR = 0.5118F;
    constexpr const float GLV_CONV_FACTOR = 0.1036F;
}

namespace thresh_default_params
{
    constexpr const float TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH = 0.2F;
    constexpr const float TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH = 3.0F;
    constexpr const float SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD = 12.0F;
}

namespace config_default_params
{
    constexpr const uint32_t IMD_STARTUP_TIME = 2000;
    constexpr const float BIT_RESOLUTION = 4095.0F;
    constexpr const float TEENSY41_MAX_INPUT_VOLTAGE = 3.3F;
}

struct ADCInterfaceParams_s
{
    pin teensy_imd_ok_pin;
    pin teensy_precharge_pin;
    pin teensy_shdn_out_pin;
    pin teensy_ts_out_filtered_pin;
    pin teensy_pack_out_filtered_pin;
    pin teensy_bspd_current_pin;
    pin teensy_scaled_24V_pin;

    float shutdown_conv_factor;
    float precharge_conv_factor;
    float pack_and_ts_out_conv_factor;
    float shdn_out_conv_factor;
    float bspd_current_conv_factor;
    float glv_conv_factor;

    float teensy41_min_digital_read_voltage_thresh;
    float teensy41_max_digital_read_voltage_thresh;
    float shutdown_voltage_digital_threshold;

    bool _in_imd_startup_period;
    uint32_t _init_millis;
    uint32_t imd_startup_time;
    float bit_resolution;
    float teensy41_max_input_voltage;
};

class ADCInterface
{
public:
    /**
     * @param imd_ok_pin INPUT - LOW represents FAULT on IMD hardware
     * @param shdn_out_pin INPUT - in FAULT state, if SHDN
    */
    ADCInterface(pin teensy_imd_ok_pin = pin_default_params::IMD_OK_PIN,
                    pin teensy_precharge_pin = pin_default_params::PRECHARGE_PIN,
                    pin teensy_shdn_out_pin = pin_default_params::SHDN_OUT_PIN,
                    pin teensy_ts_out_filtered_pin = pin_default_params::TS_OUT_FILTERED_PIN,
                    pin teensy_pack_out_filtered_pin = pin_default_params::PACK_OUT_FILTERED_PIN,
                    pin teensy_bspd_current_pin = pin_default_params::BSPD_CURRENT_PIN,
                    pin teensy_scaled_24V_pin = pin_default_params::SCALED_24V_PIN,
                    float shutdown_conv_factor = conv_default_params::SHUTDOWN_CONV_FACTOR,
                    float precharge_conv_factor = conv_default_params::PRECHARGE_CONV_FACTOR,
                    float pack_and_ts_out_conv_factor = conv_default_params::PACK_AND_TS_OUT_CONV_FACTOR,
                    float shdn_out_conv_factor = conv_default_params::SHDN_OUT_CONV_FACTOR,
                    float bspd_current_conv_factor = conv_default_params::BSPD_CURRENT_CONV_FACTOR,
                    float glv_conv_factor = conv_default_params::GLV_CONV_FACTOR,
                    float teensy41_min_digital_read_voltage_thresh = thresh_default_params::TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH,
                    float teensy41_max_digital_read_voltage_thresh = thresh_default_params::TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH,
                    float shutdown_voltage_digital_threshold = thresh_default_params::SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD,
                    uint32_t imd_startup_time = config_default_params::IMD_STARTUP_TIME,
                    float bit_resolution = config_default_params::BIT_RESOLUTION,
                    float teensy41_max_input_voltage = config_default_params::TEENSY41_MAX_INPUT_VOLTAGE):
        _params {
            teensy_imd_ok_pin,
            teensy_precharge_pin,
            teensy_shdn_out_pin,
            teensy_ts_out_filtered_pin,
            teensy_pack_out_filtered_pin,
            teensy_bspd_current_pin,
            teensy_scaled_24V_pin,
            shutdown_conv_factor,
            precharge_conv_factor,
            pack_and_ts_out_conv_factor,
            shdn_out_conv_factor,
            bspd_current_conv_factor,
            glv_conv_factor,
            teensy41_min_digital_read_voltage_thresh,
            teensy41_max_digital_read_voltage_thresh,
            shutdown_voltage_digital_threshold,
            imd_startup_time,
            bit_resolution,
            teensy41_max_input_voltage
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
    const ADCInterfaceParams_s _params = {};

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