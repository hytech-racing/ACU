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
    constexpr const float R_V1 = 0.1f; // measurement noise for V1 - fix

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
     * @param voltage_is_fresh // true if the voltage data is fresh (only happens once per good cycle)
     */
    EKFState_s update(float current, float voltage, float dt, bool voltage_is_fresh);

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
        4.181, 4.171, 4.161, 4.151, 4.141, 4.132, 4.122, 4.112, 4.102, 4.092, 
        4.083, 4.073, 4.064, 4.055, 4.046, 4.037, 4.027, 4.018, 4.009, 4.000, 
        3.991, 3.985, 3.980, 3.975, 3.969, 3.964, 3.959, 3.953, 3.948, 3.943, 
        3.937, 3.931, 3.925, 3.919, 3.913, 3.907, 3.901, 3.895, 3.890, 3.884, 
        3.878, 3.872, 3.867, 3.861, 3.856, 3.850, 3.845, 3.839, 3.834, 3.828, 
        3.823, 3.820, 3.817, 3.815, 3.812, 3.809, 3.806, 3.804, 3.801, 3.798, 
        3.796, 3.794, 3.793, 3.792, 3.790, 3.789, 3.787, 3.786, 3.785, 3.783, 
        3.782, 3.779, 3.776, 3.774, 3.771, 3.768, 3.765, 3.763, 3.760, 3.757, 
        3.755, 3.748, 3.742, 3.736, 3.730, 3.724, 3.718, 3.712, 3.706, 3.700, 
        3.694, 3.657, 3.619, 3.581, 3.544, 3.506, 3.468, 3.431, 3.393, 3.356, 
        3.318
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