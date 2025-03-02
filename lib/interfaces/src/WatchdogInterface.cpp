#include "WatchdogInterface.h"

void WatchdogInterface::init() {
    // Pin Configuration
    pinMode(_teensy_ok_pin, INPUT);
    pinMode(_teensy_wd_kick_pin, INPUT);
    pinMode(_teensy_n_latch_en_pin, INPUT);
    pinMode(_teensy_imd_ok_pin, OUTPUT);
    pinMode(_teensy_shdn_out_pin, OUTPUT);
    
    // Initial Pin States
    digitalWrite(_teensy_ok_pin, HIGH);
    digitalWrite(_teensy_wd_kick_pin, HIGH);
    digitalWrite(_teensy_n_latch_en_pin, HIGH);
    digitalWrite(_teensy_imd_ok_pin, HIGH);
    digitalWrite(_teensy_shdn_out_pin, LOW);
}

bool WatchdogInterface::get_watchdog_state(unsigned long curr_millis) {
    if ((curr_millis - _watchdog_time) > _watchdog_kick_interval) {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
    }

    return _watchdog_state;
}




