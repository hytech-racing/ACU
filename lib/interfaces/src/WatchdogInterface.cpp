#include "WatchdogInterface.h"

void WatchdogInterface::init() {
    // Pin Configuration
    pinMode(_teensy_ok_pin, OUTPUT);
    pinMode(_teensy_wd_kick_pin, OUTPUT);
    pinMode(_teensy_n_latch_en_pin, OUTPUT);
    
    // Initial Pin States for OUTPUT pins
    digitalWrite(_teensy_ok_pin, HIGH);
    digitalWrite(_teensy_wd_kick_pin, LOW); // watchdog state set to low to start
    digitalWrite(_teensy_n_latch_en_pin, LOW); 
}

bool WatchdogInterface::update_watchdog_state(uint32_t curr_millis) {
    if ((curr_millis - _watchdog_time) > _watchdog_kick_interval) {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
        digitalWrite(_teensy_wd_kick_pin, _watchdog_state ? HIGH : LOW);
    }

    return _watchdog_state;
}

void WatchdogInterface::set_teensy_ok_low() {
    digitalWrite(_teensy_ok_pin, LOW);
}

void WatchdogInterface::set_teensy_ok_high() {
    digitalWrite(_teensy_ok_pin, HIGH);
}


void WatchdogInterface::set_n_latch_en_low() {
    digitalWrite(_teensy_n_latch_en_pin, LOW);
}

void WatchdogInterface::set_n_latch_en_high() {
    digitalWrite(_teensy_n_latch_en_pin, HIGH);
}