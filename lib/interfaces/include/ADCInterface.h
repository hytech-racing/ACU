#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

struct ADCDefaultPins_s
{
    static const pin TS_OUT_FILTERED_PIN = 17;
    static const pin PACK_OUT_FILTERED_PIN = 18;
    static const pin IMD_OK_PIN = 25;
    static const pin PRECHARGE_PIN = 26;
    static const pin BSPD_CURRENT_PIN = 27;
    static const pin SHDN_OUT_PIN = 38;
    static const pin SCALED_24V_PIN = 39;
};

struct ADCConfig_s
{
    static const uint32_t imd_startup_time = 2000;
    static const float bit_resolution = 4095.0F;
    static const float teensy41_max_input_voltage = 3.3F;
    static const float shutdown_conv_factor = 0.1155F; // voltage divider -> 4.7k / (4.7k + 36k)
    static const float precharge_conv_factor = 0.6623F; // voltage divider -> 10k / (5.1k + 10k)
    static const float teensy41_min_digital_read_voltage_thresh = 0.2F;
    static const float teensy41_max_digital_read_voltage_thresh = 3.0F;
    static const float pack_and_ts_out_conv_factor = 0.00482F;
    static const float shdn_out_conv_factor = 0.1036F;
    static const float bspd_current_conv_factor = 0.5118F;
    static const float glv_conv_factor = 0.1036F;
    static const float shutdown_voltage_digital_threshold = 12.0F;
};
class ADCInterface
{
public:
    /**
     * @param imd_ok_pin INPUT - LOW represents FAULT on IMD hardware
     * @param shdn_out_pin INPUT - in FAULT state, if SHDN
    */
    ADCInterface(
        pin imd_ok_pin = ADCDefaultPins_s::IMD_OK_PIN,
        pin precharge_pin = ADCDefaultPins_s::PRECHARGE_PIN,
        pin shdn_out_pin = ADCDefaultPins_s::SHDN_OUT_PIN,
        pin ts_out_filtered_pin = ADCDefaultPins_s::TS_OUT_FILTERED_PIN,
        pin pack_out_filtered_pin = ADCDefaultPins_s::PACK_OUT_FILTERED_PIN,
        pin bspd_current_pin = ADCDefaultPins_s::BSPD_CURRENT_PIN,
        pin scaled_24V_pin = ADCDefaultPins_s::SCALED_24V_PIN):
            _teensy_imd_ok_pin(imd_ok_pin),
            _teensy_precharge_pin(precharge_pin),
            _teensy_shdn_out_pin(shdn_out_pin),
            _teensy_ts_out_filtered_pin(ts_out_filtered_pin),
            _teensy_pack_out_filtered_pin(pack_out_filtered_pin),
            _teensy_bspd_current_pin(bspd_current_pin),
            _teensy_scaled_24V_pin(scaled_24V_pin),
            _imd_startup_time(ADCConfig_s::imd_startup_time),
            _bit_resolution(ADCConfig_s::bit_resolution),
            _teensy41_max_input_voltage(ADCConfig_s::teensy41_max_input_voltage),
            _shutdown_conv_factor(ADCConfig_s::shutdown_conv_factor),
            _precharge_conv_factor(ADCConfig_s::precharge_conv_factor),
            _teensy41_min_digital_read_voltage_thresh(ADCConfig_s::teensy41_min_digital_read_voltage_thresh),
            _teensy41_max_digital_read_voltage_thresh(ADCConfig_s::teensy41_max_digital_read_voltage_thresh),
            _pack_and_ts_out_conv_factor(ADCConfig_s::pack_and_ts_out_conv_factor),
            _shdn_out_conv_factor(ADCConfig_s::shdn_out_conv_factor),
            _bspd_current_conv_factor(ADCConfig_s::bspd_current_conv_factor),
            _glv_conv_factor(ADCConfig_s::glv_conv_factor),
            _shutdown_voltage_digital_threshold(ADCConfig_s::shutdown_voltage_digital_threshold)
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
    volt read_shdn_voltage();

    /**
     * @return the state of PRECHARGE -- HIGH indicates precharge relay is closed
     */
    bool read_precharge_out();
    volt read_precharge_voltage();

    /**
     * @return voltage values of filtered TS OUT and PACK OUT
    */
    volt read_ts_out_filtered();
    volt read_pack_out_filtered();

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
    const pin _teensy_imd_ok_pin; // < READ from IMD hardware, go to FAULT state if HIGH
    const pin _teensy_precharge_pin; // READ from PRECHARGE
    const pin _teensy_shdn_out_pin; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    const pin _teensy_ts_out_filtered_pin;
    const pin _teensy_pack_out_filtered_pin;
    const pin _teensy_bspd_current_pin;
    const pin _teensy_scaled_24V_pin;

    bool _in_imd_startup_period;

    uint32_t _init_millis;
    const uint32_t _imd_startup_time;
    const float _bit_resolution;
    const float _teensy41_max_input_voltage;
    const float _shutdown_conv_factor;
    const float _precharge_conv_factor;
    const float _teensy41_min_digital_read_voltage_thresh;
    const float _teensy41_max_digital_read_voltage_thresh;
    const float _pack_and_ts_out_conv_factor;
    const float _shdn_out_conv_factor;
    const float _bspd_current_conv_factor;
    const float _glv_conv_factor;
    const float _shutdown_voltage_digital_threshold;
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif