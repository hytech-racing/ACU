#ifndef SOC_KALMAN_FILTER_H
#define SOC_KALMAN_FILTER_H

#include <array>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/singleton.h"
#include "SharedFirmwareTypes.h"
#include "shared_types.h"

namespace soc_ekf_constants
{
    constexpr const float CAPACITY_AS = 48600.0f; // Pack capacity in Amp-seconds (13.5 Ah * 3600 s/h)
    constexpr const float R0 = 0.00195238095f; // Internal series resistance (instantaneous voltage drop)
    constexpr const float R1 = 0.00039047619f; // Polarization resistance to see slow voltage response - fix
    constexpr const float TIME_CONSTANT = 20.0f; // time constant value - fix
    constexpr const float C1 = TIME_CONSTANT / R1; // Polarization capacitance

    // EKF tuning parameters (update these to tune the EKF to track SoC better)
    constexpr const float Q_SOC = 1e-5f; // process noise for SoC - fix
    constexpr const float Q_V1 = 1e-6f; // process noise for V1 - fix
    constexpr const float R_V1 = 0.001f; // measurement noise for V1 - fix

    constexpr const float MIN_SOC = 0.0f;
    constexpr const float MAX_SOC = 1.0f;
    constexpr const float MAX_V1_MAGNITUDE = 0.5f;

    // Initial/reset values for state and covariance
    constexpr const float INITIAL_SOC = 0.5f;
    constexpr const float INITIAL_V1 = 0.0f;
    constexpr const float P_SOC_INITIAL = 0.01f;  // Initial variance for SoC
    constexpr const float P_V1_INITIAL = 0.1f;    // Initial variance for V1
    constexpr const float P_CROSS_INITIAL = 0.0f; // Initial cross-covariance

    // Numerical differentiation step size
    constexpr const float DOCV_DSOC_STEP = 0.01f;
    // Minimum slope for dOCV/dSoC to prevent numerical instability
    constexpr const float MIN_DOCV_DSOC_SLOPE = 0.1f;
    constexpr const float MIN_INNOVATION_COV_THRESH = 1e-6f;
    constexpr const float DIVIDER_CROSS_AVG = 2.0f;

    constexpr const float P_SOC_AFTER_REST = 0.001f;
    constexpr const float P_V1_AFTER_REST = 0.001f;
}

struct EKFState_s
{
    float soc; // State of charge state varying from 0.0 to 1.0
    float v1; // Polarization voltage to track lag in voltage from current change
};

class SOCKalmanFilter
{
public:
    SOCKalmanFilter();

    /**
     * @brief Set the EKF state appropriately based on the initial voltage
     * This is called when the ACU starts and is the first reading done on the EKF
     * @param initial_voltage // this is the voltage coming in from the cells on the pack (we take the minimum voltage for a per cell EKF)
     */
    void init(float initial_voltage);

    /**
     * @brief Used to update the state of our EKF at specified time intervals
     * @param current // current going across the pack in amps
     * @param voltage // minimum cell voltage across the pack
     * @param dt // time elapsed since last update in seconds
     */
    EKFState_s update(float current, float voltage, float dt);

    /**
     * @brief Get the soc object
     * @return float state of charge
     */
    float get_soc() const {
        return _state.soc;
    }

    /**
     * @brief Get current state
     * @return Complete state vector
     */
    EKFState_s get_state() const {
        return _state;
    }

    /**
     * @brief Reset SoC estimate
     * @param new_soc New SoC value
     * @post SoC updated, uncertainty increased
     */
    void reset_soc(float new_soc);

    // OCV Lookup Table
    // Index 0 = 100% SoC (4.2V approx), Index 100 = 0% SoC (3.585V approx)
    // fix tuned
    static constexpr float VOLTAGE_LOOKUP_TABLE[101] = {
        4.197, 4.188, 4.179, 4.170, 4.160, 4.151, 4.141, 4.132, 4.122, 4.113, 
        4.103, 4.093, 4.082, 4.071, 4.060, 4.049, 4.038, 4.028, 4.018, 4.010, 
        4.002, 3.995, 3.990, 3.984, 3.979, 3.975, 3.970, 3.965, 3.961, 3.955, 
        3.950, 3.944, 3.938, 3.932, 3.925, 3.918, 3.912, 3.905, 3.899, 3.892, 
        3.886, 3.880, 3.873, 3.867, 3.861, 3.855, 3.849, 3.843, 3.837, 3.832, 
        3.828, 3.824, 3.820, 3.816, 3.813, 3.810, 3.807, 3.804, 3.801, 3.798, 
        3.796, 3.794, 3.792, 3.790, 3.788, 3.787, 3.785, 3.783, 3.782, 3.780, 
        3.778, 3.776, 3.774, 3.772, 3.770, 3.767, 3.765, 3.763, 3.760, 3.757, 
        3.754, 3.751, 3.747, 3.743, 3.739, 3.735, 3.730, 3.724, 3.719, 3.713, 
        3.706, 3.698, 3.690, 3.680, 3.669, 3.657, 3.644, 3.630, 3.616, 3.601, 
        3.585
    };

private:
    /**
     * @brief Get the OCV from the SoC estimate using linear interpolation of lookup table
     * @param soc State of charge
     * @return Open circuit voltage
     */
    float _get_ocv_from_soc(float soc) const;

    /**
     * @brief Get dOCV/dSoC for Jacobian in the EKF calculation
     * @param soc State of charge
     * @return Numerical derivative
     */
    float _get_docv_dsoc(float soc) const;

    /**
     * @brief Clamp state to physical limits
     */
    void _clamp_state();

private:
    EKFState_s _state; // The system state (SoC, V_polarization)

    // Covariance Matrix P (2x2)
    // Tracks the uncertainty of our estimate.
    // P[0][0] = var(SoC), P[1][1] = var(V1)
    float _PMatrix[2][2];
};

#endif