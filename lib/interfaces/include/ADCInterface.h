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
    constexpr const pin IMD_OK_PIN = 25;
    constexpr const pin PRECHARGE_PIN = 26;
    constexpr const pin BSPD_CURRENT_PIN = 27;
    constexpr const pin SHDN_OUT_PIN = 38;
    constexpr const pin SCALED_24V_PIN = 39;
};

class ADCInterface
{
public:
    /**
     * @param imd_ok_pin INPUT - LOW represents FAULT on IMD hardware
     * @param shdn_out_pin INPUT - in FAULT state, if SHDN
    */
    ADCInterface(
        pin imd_ok_pin = pin_default_params::IMD_OK_PIN,
        pin precharge_pin = pin_default_params::PRECHARGE_PIN,
        pin shdn_out_pin = pin_default_params::SHDN_OUT_PIN,
        pin ts_out_filtered_pin = pin_default_params::TS_OUT_FILTERED_PIN,
        pin pack_out_filtered_pin = pin_default_params::PACK_OUT_FILTERED_PIN,
        pin bspd_current_pin = pin_default_params::BSPD_CURRENT_PIN,
        pin scaled_24V_pin = pin_default_params::SCALED_24V_PIN):
            _teensy_imd_ok_pin(imd_ok_pin),
            _teensy_shdn_out_pin(shdn_out_pin),
            _teensy_precharge_pin(precharge_pin),
            _teensy_ts_out_filtered_pin(ts_out_filtered_pin),
            _teensy_pack_out_filtered_pin(pack_out_filtered_pin),
            _teensy_bspd_current_pin(bspd_current_pin),
            _teensy_scaled_24V_pin(scaled_24V_pin)
    {};

    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
    */ 
    void init(uint32_t init_millis);

private:
    const pin _teensy_imd_ok_pin; // < READ from IMD hardware, go to FAULT state if HIGH
    const pin _teensy_precharge_pin; // READ from PRECHARGE
    const pin _teensy_shdn_out_pin; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    const pin _teensy_ts_out_filtered_pin;
    const pin _teensy_pack_out_filtered_pin;
    const pin _teensy_bspd_current_pin;
    const pin _teensy_scaled_24V_pin;

    bool _in_imd_startup_period;

    uint32_t _init_millis = 0;
    const uint32_t _imd_startup_time = 2000;
    const float _bit_resolution = 4095.0F;
    const float _teensy41_max_input_voltage = 3.3F;
    const float _shutdown_conv_factor = 0.1155F; // voltage divider -> 4.7k / (4.7k + 36k)
    const float _precharge_conv_factor = 0.6623F; // voltage divider -> 10k / (5.1k + 10k)
    const float _teensy41_min_digital_read_voltage_thresh = 0.2F;
    const float _teensy41_max_digital_read_voltage_thresh = 3.0F;
    const float _pack_and_ts_out_conv_factor = 0.00482F;
    const float _shdn_out_conv_factor = 0.1036F;
    const float _bspd_current_conv_factor = 0.5118F;
    const float _glv_conv_factor = 0.1036F;
    const float _shutdown_voltage_digital_threshold = 12.0F;
public:
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
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif