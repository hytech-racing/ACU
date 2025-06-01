#include "WatchdogInterface.h"

void WatchdogInterface::init(uint32_t init_millis) {
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
    digitalWrite(_teensy_n_latch_en_pin, LOW); 
    _init_millis = init_millis;
    _in_imd_startup_period = true; 
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

bool WatchdogInterface::read_imd_ok(uint32_t curr_millis) {
    if (_in_imd_startup_period) {
        if ((curr_millis - _init_millis) >= _imd_startup_time) { // give 2 seconds for IMD to startup
            _in_imd_startup_period = false;
        }
        return true;
    }
    return (static_cast<float>(analogRead(_teensy_imd_ok_pin)) * (_teensy41_max_input_voltage / _bit_resolution)) > _teensy41_min_digital_read_voltage_thresh; // idk if this would actually work, like if a LOW is a threshold or smth
}

bool WatchdogInterface::read_shdn_out() {
    bool data = digitalRead(_teensy_shdn_out_pin);
    return data;
}

volt WatchdogInterface::read_ts_out_filtered() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_teensy_ts_out_filtered_pin)) * (_teensy41_max_input_voltage / _bit_resolution) / (_pack_and_ts_out_conv_factor); 
    return data;
}

volt WatchdogInterface::read_pack_out_filtered() {
    volt data = static_cast<float>(analogRead(_teensy_pack_out_filtered_pin)) * (_teensy41_max_input_voltage / _bit_resolution) / (_pack_and_ts_out_conv_factor);
    return data;
}

volt WatchdogInterface::read_bspd_current() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_teensy_bspd_current_pin)) * (_teensy41_max_input_voltage / _bit_resolution) / _bspd_current_conv_factor; //((24.16 / 5.0) * (4.3 / 36.0)); 
    return data;
}

volt WatchdogInterface::read_global_lv_value() {
    volt data = static_cast<float>(analogRead(_teensy_scaled_24V_pin)) * (_teensy41_max_input_voltage / _bit_resolution) / (_glv_conv_factor); // input before voltage divider (4.3k / (4.3k + 36k))
    return data;
}
