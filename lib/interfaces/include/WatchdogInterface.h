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
};

class WatchdogInterface
{
public:
    /**
     * @param _teensy_ok_pin OUTPUT - needs to stay HIGH, the other input to Watchdog on ACU
     * @param wd_kick_pin OUTPUT - 100 Hz pulse from teensy, one of inputs to Watchdog hardware on ACU 
     * @param n_latch_en_pin OUTPUT - should be HIGH if NOT Faulted
    */
    WatchdogInterface(
        pin teensy_ok_pin = pin_default_params::TEENSY_OK_PIN,
        pin wd_kick_pin = pin_default_params::WD_KICK_PIN, 
        pin n_latch_pin = pin_default_params::N_LATCH_EN_PIN,
        const uint32_t kick_interval_ms = 9UL) : 
                        _teensy_wd_kick_pin(wd_kick_pin),
                        _teensy_ok_pin(teensy_ok_pin),
                        _teensy_n_latch_en_pin(n_latch_pin),
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
    const pin _teensy_wd_kick_pin;  // > Needs to flip at 100 Hz to keep BMS_OK high
    const pin _teensy_ok_pin;   // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    const pin _teensy_n_latch_en_pin; // > Input to Safety Light, true when teensy is not in FAULT state

    // following is taken from VCR Watchdog System
    /* Watchdog last kicked time and output state */
    const uint32_t _watchdog_kick_interval;
    uint32_t _watchdog_time;                                 
    bool _watchdog_state;

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
};

using WatchdogInstance = etl::singleton<WatchdogInterface>;

#endif