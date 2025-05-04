#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

namespace WATCHDOG_DEFAULT_PARAMS
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
        pin teensy_ok_pin = WATCHDOG_DEFAULT_PARAMS::TEENSY_OK_PIN,
        pin wd_kick_pin = WATCHDOG_DEFAULT_PARAMS::WD_KICK_PIN, 
        pin n_latch_pin = WATCHDOG_DEFAULT_PARAMS::N_LATCH_EN_PIN,
        pin imd_ok_pin = WATCHDOG_DEFAULT_PARAMS::IMD_OK_PIN, 
        pin shdn_out_pin = WATCHDOG_DEFAULT_PARAMS::SHDN_OUT_PIN,
        pin ts_out_filtered_pin = WATCHDOG_DEFAULT_PARAMS::TS_OUT_FILTERED_PIN,
        pin pack_out_filtered_pin = WATCHDOG_DEFAULT_PARAMS::PACK_OUT_FILTERED_PIN,
        pin bspd_current_pin = WATCHDOG_DEFAULT_PARAMS::BSPD_CURRENT_PIN,
        pin scaled_24V_pin = WATCHDOG_DEFAULT_PARAMS::SCALED_24V_PIN,
        const uint32_t kick_interval_ms = 10UL) : 
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
    void init();

private:
    /* Pin Assignments */
    pin _teensy_wd_kick_pin;  // > Needs to flip at 100 Hz to keep BMS_OK high
    pin _teensy_ok_pin;   // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    pin _teensy_n_latch_en_pin; // > Input to Safety Light, true when teensy is not in FAULT state
    pin _teensy_imd_ok_pin; // < READ from IMD hardware, go to FAULT state if HIGH
    pin _teensy_shdn_out_pin; // < READ from SHDN hardware, can leave FAULT state if goes to HIGH to signify car startup
    pin _teensy_ts_out_filtered_pin;
    pin _teensy_pack_out_filtered_pin;
    pin _teensy_bspd_current_pin;
    pin _teensy_scaled_24V_pin;

    // following is taken from VCR Watchdog System
    /* Watchdog last kicked time */
    uint32_t _watchdog_kick_interval;
    uint32_t _watchdog_time;
    bool _watchdog_state;
    /* Watchdog output state */

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
    bool read_imd_ok();

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