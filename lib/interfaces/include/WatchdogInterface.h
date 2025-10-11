#ifndef WATCHDOG_INTERFACE_H
#define WATCHDOG_INTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "../../../include/ACU_Constants.h"
#include "etl/singleton.h"

using pin = size_t;

struct WatchdogInterfaceParams_s
{
    pin teensy_ok_pin; // > Needs to stay HIGH while wd_kick_pin flips to keep BMS_OK high
    pin teensy_wd_kick_pin; // > Needs to flip at 100 Hz to keep BMS_OK high
    pin teensy_n_latch_en_pin; // > Input to Safety Light, true when teensy is not in FAULT state
    uint32_t watchdog_kick_interval;
};

namespace pin_default_params
{
    constexpr const WatchdogInterfaceParams_s WATCHDOG_PINOUT = {ACUConstants::TEENSY_OK_PIN, 
                                                            ACUConstants::WD_KICK_PIN, 
                                                            ACUConstants::N_LATCH_EN_PIN,
                                                            ACUConstants::WATCHDOG_KICK_INTERVAL};
}

class WatchdogInterface
{
public:
    WatchdogInterface(WatchdogInterfaceParams_s params = {pin_default_params::WATCHDOG_PINOUT}):
        _watchdog_params {
            params
        }
        {};
    
    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
    */ 
    void init();

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

private:
    const WatchdogInterfaceParams_s _watchdog_params = {};

    // @brief timestamp of the last watchdog kick
    uint32_t _watchdog_time = 0;

    // @brief current output level driven on the watchdog kick pin, true = HIGH
    bool _watchdog_state = false;
};

using WatchdogInstance = etl::singleton<WatchdogInterface>;

#endif