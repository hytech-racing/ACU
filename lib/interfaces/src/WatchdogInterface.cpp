#include "WatchdogInterface.h"

void WatchdogInterface::init() {
    // Pin Configuration
    pinMode(_teensy_ok_pin, OUTPUT);
    pinMode(_teensy_wd_kick_pin, OUTPUT);
    pinMode(_teensy_n_latch_en_pin, OUTPUT);
    pinMode(_teensy_imd_ok_pin, INPUT);
    pinMode(_teensy_shdn_out_pin, INPUT);
    pinMode(_teensy_ts_out_filtered_pin, INPUT);
    pinMode(_teensy_pack_out_filtered_pin, INPUT);
    
    // Initial Pin States for OUTPUT pins
    digitalWrite(_teensy_ok_pin, HIGH);
    digitalWrite(_teensy_wd_kick_pin, LOW); // watchdog state set to low to start
    digitalWrite(_teensy_n_latch_en_pin, HIGH); 

    analogReadResolution(12);
}

bool WatchdogInterface::update_watchdog_state(unsigned long curr_millis) {
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

bool WatchdogInterface::read_imd_ok() {
    bool data = digitalRead(_teensy_imd_ok_pin);
    return data; // idk if this would actually work, like if a LOW is a threshold or smth
}

bool WatchdogInterface::read_shdn_out() {
    bool data = digitalRead(_teensy_shdn_out_pin);
    return data;
}

volt WatchdogInterface::read_ts_out_filtered() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_teensy_ts_out_filtered_pin)) * (3.3 / 4095.0); 
    return data;
}

volt WatchdogInterface::read_pack_out_filtered() {
    volt data = static_cast<float>(analogRead(_teensy_pack_out_filtered_pin)) * (3.3 / 4095.0);
    return data;
}


