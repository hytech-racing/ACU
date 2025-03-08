#include "WatchdogInterface.h"
#include "WatchdogInterface.h"

void WatchdogInterface::init() {
    // Pin Configuration
    pinMode(_teensy_ok_pin, OUTPUT);
    pinMode(_teensy_wd_kick_pin, OUTPUT);
    pinMode(_teensy_n_latch_en_pin, OUTPUT);
    pinMode(_teensy_imd_ok_pin, INPUT);
    pinMode(_teensy_shdn_out_pin, INPUT);
    
    // Initial Pin States
    digitalWrite(_teensy_ok_pin, HIGH);
    digitalWrite(_teensy_wd_kick_pin, LOW); // watchdog state set to low to start
    digitalWrite(_teensy_n_latch_en_pin, HIGH); 
}

bool WatchdogInterface::get_watchdog_state(unsigned long curr_millis) {
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

void WatchdogInterface::set_n_latch_en_low() {
    digitalWrite(_teensy_n_latch_en_pin, LOW);
}

bool WatchdogInterface::read_imd_ok() {
    uint8_t data = digitalRead(_teensy_imd_ok_pin);
    return data != 0; // idk if this would actually work, like if a LOW is a threshold or smth
}

bool WatchdogInterface::read_shdn_out() {
    uint8_t data = digitalRead(_teensy_shdn_out_pin);
    return data != 0;
}


