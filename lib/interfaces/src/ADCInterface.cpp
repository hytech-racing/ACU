#include "ADCInterface.h"

void ADCInterface::init(uint32_t init_millis) {
    // Pin Configuration
    pinMode(_adc_parameters.pinout.teensy_imd_ok_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_precharge_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_out_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_ts_out_filtered_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_pack_out_filtered_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_bspd_current_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_scaled_24V_pin, INPUT);

    _init_millis = init_millis;
    _in_imd_startup_period = true; 
}

bool ADCInterface::read_imd_ok(uint32_t curr_millis) {
    if (_in_imd_startup_period) {
        if ((curr_millis - _init_millis) >= _adc_parameters.configs.imd_startup_time) { // give 2 seconds for IMD to startup
            _in_imd_startup_period = false;
        }
        return true;
    }
    //
    return static_cast<float>(analogRead(_adc_parameters.pinout.teensy_imd_ok_pin)) * (_adc_parameters.configs.teensy41_max_input_voltage / _adc_parameters.bit_resolution) > _adc_parameters.thresholds.teensy41_min_digital_read_voltage_thresh; // idk if this would actually work, like if a LOW is a threshold or smth
}

volt ADCInterface::read_shdn_voltage() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_shdn_out_pin)) * _adc_parameters.conversions.shutdown_conv_factor;
    return data;
}

bool ADCInterface::read_shdn_out() {
    bool data = read_shdn_voltage() > _adc_parameters.thresholds.shutdown_voltage_digital_threshold; // read shdn out goes high if shutdown voltage is > 12 volts
    return data;
}

volt ADCInterface::read_shdn_out_voltage() {
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_shdn_out_pin)) * _adc_parameters.conversions.shdn_out_conv_factor;
    return data;
}

volt ADCInterface::read_precharge_voltage() {
    // 3.3 V for pin voltage cap
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_precharge_pin)) * _adc_parameters.conversions.precharge_conv_factor;
    return data;
}

bool ADCInterface::read_precharge_out() {
    bool data = read_precharge_voltage() > _adc_parameters.thresholds.teensy41_min_digital_read_voltage_thresh;
    return data;
}

volt ADCInterface::read_ts_out_filtered() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_ts_out_filtered_pin)) * _adc_parameters.conversions.pack_and_ts_out_conv_factor;
    return data;
}

volt ADCInterface::read_pack_out_filtered() {
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_pack_out_filtered_pin)) * _adc_parameters.conversions.pack_and_ts_out_conv_factor;
    return data;
}

volt ADCInterface::read_bspd_current() {
    // 3.3 V for pin voltage cap. and 4095 for bit resolution
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_bspd_current_pin)) * _adc_parameters.conversions.bspd_current_conv_factor; //((24.16 / 5.0) * (4.3 / 36.0));
    return data;
}

volt ADCInterface::read_global_lv_value() {
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_scaled_24V_pin)) * _adc_parameters.conversions.glv_conv_factor; // input before voltage divider (4.3k / (4.3k + 36k))
    return data;
}

const ADCInterfaceParams_s& ADCInterface::get_adc_params() const {
    return _adc_parameters;
}

bool ADCInterface::is_in_imd_startup_period() const {
    return _in_imd_startup_period;
}