#ifndef WATCHDOG_INTERFACE_H
#define WATCHDOG_INTERFACE_H

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

using pin = size_t;

namespace watchdog_default_parameters
{
    constexpr const uint32_t WATCHDOG_KICK_INTERVAL = 9UL;
}

struct WatchdogPinout_s
{
    pin teensy_ok_pin; 
    pin teensy_wd_kick_pin;
    pin teensy_n_latch_en_pin;
};
struct WatchdogInterfaceParams_s
{
    WatchdogPinout_s pinout;
    uint32_t watchdog_kick_interval;
};

class WatchdogInterface
{
public:
    WatchdogInterface(WatchdogPinout_s pinout,
                        uint32_t watchdog_kick_interval = watchdog_default_parameters::WATCHDOG_KICK_INTERVAL
    ): _watchdog_parameters { 
            pinout, 
            watchdog_kick_interval} {}
    
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
    const WatchdogInterfaceParams_s _watchdog_parameters = {};

    // @brief timestamp of the last watchdog kick
    uint32_t _watchdog_time = 0;

    // @brief current output level driven on the watchdog kick pin, true = HIGH
    bool _watchdog_state = false;
};

using WatchdogInstance = etl::singleton<WatchdogInterface>;

#endif