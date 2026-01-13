#include "SOCKalmanFilter.h"
#include <math.h>

SOCKalmanFilter::SOCKalmanFilter() {
    _state.soc = 0.5f;
    _state.v1 = 0.0f;

    _PMatrix[0][0] = 0.01f;
    _PMatrix[0][1] = 0.0f;
    _PMatrix[1][0] = 0.0f;
    _PMatrix[1][1] = 0.1f;
}

void SOCKalmanFilter::init(float initial_voltage) {
    static constexpr size_t table_size = 101;

    if (initial_voltage >= _VOLTAGE_LOOKUP_TABLE[0]) {
        _state.soc = 1.0f;
    } else if (initial_voltage <= _VOLTAGE_LOOKUP_TABLE[table_size - 1]) {
        _state.soc = 0.0f;
    } else {
        for (size_t i = 0; i < table_size - 1; i++) {
            if (initial_voltage <= _VOLTAGE_LOOKUP_TABLE[i] && initial_voltage > _VOLTAGE_LOOKUP_TABLE[i + 1]) {
                float v_high = _VOLTAGE_LOOKUP_TABLE[i];
                float v_low = _VOLTAGE_LOOKUP_TABLE[i + 1];
                float soc_high = (float)(table_size - 1 - i) / (table_size - 1);
                float soc_low = (float)(table_size - 1 - (i + 1)) / (table_size - 1);
                
                _state.soc = soc_low + (initial_voltage - v_low) / (v_high - v_low) * (soc_high - soc_low);
                break;
            }
        }

    }

    _state.v1 = 0.0f;
    
    _PMatrix[0][0] = 0.01f;
    _PMatrix[0][1] = 0.0f;
    _PMatrix[1][0] = 0.0f;
    _PMatrix[1][1] = 0.1f;
}

EKFState_s SOCKalmanFilter::update(float current, float voltage, float dt) {
    // Prediction
    float soc_rate = -current / soc_ekf_constants::CAPACITY_AS;
    _state.soc += soc_rate * dt;

    float decay_factor = expf(-dt / soc_ekf_constants::TIME_CONSTANT);
    _state.v1 = _state.v1 * decay_factor + current * soc_ekf_constants::R1 * (1.0f - decay_factor);

    _clamp_state();

    // Predict covariance (F * P * F^T + Q)
    float F11 = decay_factor;

    float FP00 = _PMatrix[0][0];
    float FP01 = _PMatrix[0][1];
    float FP10 = F11 * _PMatrix[1][0];
    float FP11 = F11 * _PMatrix[1][1];

    _PMatrix[0][0] = FP00 + soc_ekf_constants::Q_SOC;
    _PMatrix[0][1] = FP01 * F11;
    _PMatrix[1][0] = FP10;
    _PMatrix[1][1] = FP11 * F11 + soc_ekf_constants::Q_V1;

    // Update
    float ocv = _get_ocv_from_soc(_state.soc);
    float voltage_pred = ocv - current * soc_ekf_constants::R0 - _state.v1;
    float innovation = voltage - voltage_pred;

    float H0 = _get_docv_dsoc(_state.soc);
    float H1 = -1.0f;

    float HP0 = H0 * _PMatrix[0][0] + H1 * _PMatrix[1][0];
    float HP1 = H0 * _PMatrix[0][1] + H1 * _PMatrix[1][1];
    float S = HP0 * H0 + HP1 * H1 + soc_ekf_constants::R_V1;
    
    float K0 = (_PMatrix[0][0] * H0 + _PMatrix[0][1] * H1) / S;
    float K1 = (_PMatrix[1][0] * H0 + _PMatrix[1][1] * H1) / S;
    
    _state.soc += K0 * innovation;
    _state.v1 += K1 * innovation;
    _clamp_state();

    float I_KH_00 = 1.0f - K0 * H0;
    float I_KH_01 = -K0 * H1;
    float I_KH_10 = -K1 * H0;
    float I_KH_11 = 1.0f - K1 * H1;
    
    float P00_new = I_KH_00 * _PMatrix[0][0] + I_KH_01 * _PMatrix[1][0];
    float P01_new = I_KH_00 * _PMatrix[0][1] + I_KH_01 * _PMatrix[1][1];
    float P10_new = I_KH_10 * _PMatrix[0][0] + I_KH_11 * _PMatrix[1][0];
    float P11_new = I_KH_10 * _PMatrix[0][1] + I_KH_11 * _PMatrix[1][1];
    
    _PMatrix[0][0] = P00_new;
    _PMatrix[0][1] = P01_new;
    _PMatrix[1][0] = P10_new;
    _PMatrix[1][1] = P11_new;
    
    return _state;
}

void SOCKalmanFilter::_clamp_state() {
    if (_state.soc < soc_ekf_constants::MIN_SOC) {
        _state.soc = soc_ekf_constants::MIN_SOC;
    }
    if (_state.soc > soc_ekf_constants::MAX_SOC) {
        _state.soc = soc_ekf_constants::MAX_SOC;
    }
    if (_state.v1 < -soc_ekf_constants::MAX_V1_MAGNITUDE) {
        _state.v1 = -soc_ekf_constants::MAX_V1_MAGNITUDE;
    }
    if (_state.v1 > soc_ekf_constants::MAX_V1_MAGNITUDE) {
        _state.v1 = soc_ekf_constants::MAX_V1_MAGNITUDE;
    }
}

float SOCKalmanFilter::_get_ocv_from_soc(float soc) const {
    static constexpr size_t table_size = 101;
    
    if (soc >= 1.0f) {
        return _VOLTAGE_LOOKUP_TABLE[0];
    }
    if (soc <= 0.0f) {
        return _VOLTAGE_LOOKUP_TABLE[table_size - 1];
    }
    
    float index_float = (1.0f - soc) * 100.0f;
    size_t idx_low = (size_t)index_float;
    size_t idx_high = idx_low + 1;

    if (idx_high >= table_size) {
        idx_high = table_size - 1;
        idx_low = idx_high - 1;
    }

    float fraction = index_float - (float)idx_low;
    return _VOLTAGE_LOOKUP_TABLE[idx_low] + fraction * (_VOLTAGE_LOOKUP_TABLE[idx_high] - _VOLTAGE_LOOKUP_TABLE[idx_low]);
}

float SOCKalmanFilter::_get_docv_dsoc(float soc) const {
    constexpr float h = 0.01f;
    float soc_plus = fminf(soc + h, 1.0f);
    float soc_minus = fmaxf(soc - h, 0.0f);
    float ocv_plus = _get_ocv_from_soc(soc_plus);
    float ocv_minus = _get_ocv_from_soc(soc_minus);
    return (ocv_plus - ocv_minus) / (soc_plus - soc_minus);
}

void SOCKalmanFilter::reset_soc(float new_soc) {
    _state.soc = fmaxf(soc_ekf_constants::MIN_SOC, fminf(soc_ekf_constants::MAX_SOC, new_soc));
    _PMatrix[0][0] = 0.01f;
    _PMatrix[0][1] = 0.0f;
    _PMatrix[1][0] = 0.0f;
    _PMatrix[1][1] = 0.1f;
}