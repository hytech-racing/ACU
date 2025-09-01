#ifndef WATCHDOG_INTERFACE_H
#define WATCHDOG_INTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

namespace pin_default_params
{
    constexpr const pin TEENSY_OK_PIN = 3;
    constexpr const pin WD_KICK_PIN = 4;       
    constexpr const pin N_LATCH_EN_PIN = 6;
    constexpr const pin TS_OUT_FILTERED_PIN = 17;
    constexpr const pin PACK_OUT_FILTERED_PIN = 18;
    constexpr const pin IMD_OK_PIN = 25;
    constexpr const pin BSPD_CURRENT_PIN = 27;
    constexpr const pin SHDN_OUT_PIN = 38;
    constexpr const pin SCALED_24V_PIN = 39;
};

class WatchdogInterface
{
public:
    /**
     * @param _teensy_ok_pin OUTPUT - needs to stay HIGH, the other input to Watchdog on ACU
     * @param wd_kick_pin OUTPUT - 100 Hz pulse from teensy, one of inputs to Watchdog hardware on ACU 
     * @param n_latch_en_pin OUTPUT - should be HIGH if NOT Faulted
     * @param imd_ok_pin INPUT - LOW represents FAULT on IMD hardware
     * @param shdn_out_pin INPUT - in FAULT state, if SHDN
    */
    WatchdogInterface(
        pin teensy_ok_pin = pin_default_params::TEENSY_OK_PIN,
        pin wd_kick_pin = pin_default_params::WD_KICK_PIN, 
        pin n_latch_pin = pin_default_params::N_LATCH_EN_PIN,
        pin imd_ok_pin = pin_default_params::IMD_OK_PIN, 
        pin shdn_out_pin = pin_default_params::SHDN_OUT_PIN,
        pin ts_out_filtered_pin = pin_default_params::TS_OUT_FILTERED_PIN,
        pin pack_out_filtered_pin = pin_default_params::PACK_OUT_FILTERED_PIN,
        pin bspd_current_pin = pin_default_params::BSPD_CURRENT_PIN,
        pin scaled_24V_pin = pin_default_params::SCALED_24V_PIN,
        const uint32_t kick_interval_ms = 9UL) : 
                        _teensy_wd_kick_pin(wd_kick_pin),
                        _teensy_ok_pin(teensy_ok_pin),
                        _teensy_n_latch_en_pin(n_latch_pin),
                        _teensy_imd_ok_pin(imd_ok_pin),
                        _teensy_shdn_out_pin(shdn_out_pin),
                        _teensy_ts_out_filtered_pin(ts_out_filtered_pin),
                        _teensy_pack_out_filtered_pin(pack_out_filtered_pin),
                        _teensy_bspd_current_pin(bspd_current_pin),
                        _teensy_scaled_24V_pin(scaled_24V_pin),
                        _watchdog_kick_interval(kick_interval_ms),
                        _watchdog_time(0), 
                        _watchdog_state(false)
    {};
    
    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
    */ 
    void init(uint32_t init_millis);

private:
    /* Pin Assignments */
    const pin _teensy_wd_kick_pin;  // > Needs to flip at 100 Hz to keep BMS_OK high
    const pin _teensy_ok_pin;   // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    const pin _teensy_n_latch_en_pin; // > Input to Safety Light, true when teensy is not in FAULT state
    const pin _teensy_imd_ok_pin; // < READ from IMD hardware, go to FAULT state if HIGH
    const pin _teensy_shdn_out_pin; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    const pin _teensy_ts_out_filtered_pin;
    const pin _teensy_pack_out_filtered_pin;
    const pin _teensy_bspd_current_pin;
    const pin _teensy_scaled_24V_pin;

    // following is taken from VCR Watchdog System
    /* Watchdog last kicked time and output state */
    const uint32_t _watchdog_kick_interval;
    uint32_t _watchdog_time;
    bool _watchdog_state;
    bool _in_imd_startup_period;
    
    uint32_t _init_millis = 0;
    const uint32_t _imd_startup_time = 2000;
    const float _bit_resolution = 4095.0F;
    const float _teensy41_max_input_voltage = 3.3F;
    const float _teensy41_min_digital_read_voltage_thresh = 0.2F;
    const float _teensy41_max_digital_read_voltage_thresh = 3.0F;
    const float _pack_and_ts_out_conv_factor = 0.00482F;
    const float _bspd_current_conv_factor = 0.5118F;
    const float _glv_conv_factor = 0.1036F;

public: 
    /**
     * Get/update watchdog state
     * @param curr_millis time of ACU time
     * @post IF reach interval, _watchdog_time updated and state switched
    */
    bool update_watchdog_state(uint32_t curr_millis);

    /**
     * Sets Teensy_OK LOW
     * @pre ACU Controller discovers bms voltage/temp fault
    */
    void set_teensy_ok_low();
    void set_teensy_ok_high();

    /**
     * Sets n_latch_en low
     * @pre that there is a fault of some sort
    */
    void set_n_latch_en_low();
    void set_n_latch_en_high();

    /**
     * @return the state of the IMD, HIGH = NO FAULT
    */
    bool read_imd_ok(uint32_t curr_millis);

    /**
     * @return the state of SHDN_OUT, HIGH = CAR was STARTED UP
    */
    bool read_shdn_out();

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
};

using WatchdogInstance = etl::singleton<WatchdogInterface>;

#endif