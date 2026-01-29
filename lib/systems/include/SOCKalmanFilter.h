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
    constexpr const float R0 = 0.00195238095f; // Internal series resistance (instantaneous voltage drop) - fix
    constexpr const float R1 = 0.00039047619f; // Polarization resistance to see slow voltage response - fix
    constexpr const float TIME_CONSTANT = 20.0f; // time constant value
    constexpr const float C1 = TIME_CONSTANT / R1; // Polarization capacitance

    // EKF tuning parameters (update these to tune the EKF to track SoC better)
    constexpr const float Q_SOC = 1e-5f; // process noise for SoC
    constexpr const float Q_V1 = 1e-6f; // process noise for V1
    constexpr const float R_V1 = 0.001f; // measurement noise for V1

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
     * @param initial_voltage // this is the voltage coming in from the cells on the pack (we take the average voltage for a per cell EKF)
     */
    void init(float initial_voltage);

    /**
     * @brief Used to update the state of our EKF at specified time intervals
     * @param current // current going across the pack in amps
     * @param voltage // average cell voltage across the pack
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

    // OCV Lookup Table
    // Index 0 = 100% SoC (4.2V approx), Index 100 = 0% SoC (3.0V approx)
    // fix tuned
    static constexpr float _VOLTAGE_LOOKUP_TABLE[101] = {
        3.972, 3.945, 3.918, 3.891, 3.885, 3.874, 3.864, 3.858, 3.847, 3.836,
        3.820, 3.815, 3.815, 3.798, 3.788, 3.782, 3.771, 3.755, 3.744, 3.744,
        3.733, 3.728, 3.723, 3.712, 3.701, 3.695, 3.690, 3.679, 3.679, 3.668,
        3.663, 3.657, 3.647, 3.647, 3.636, 3.625, 3.625, 3.625, 3.614, 3.609,
        3.603, 3.603, 3.592, 3.592, 3.592, 3.581, 3.581, 3.571, 3.571, 3.571,
        3.560, 3.560, 3.560, 3.549, 3.549, 3.549, 3.549, 3.538, 3.538, 3.551,
        3.546, 3.535, 3.535, 3.535, 3.530, 3.524, 3.524, 3.524, 3.513, 3.513,
        3.513, 3.503, 3.503, 3.492, 3.492, 3.492, 3.487, 3.481, 3.481, 3.476,
        3.471, 3.460, 3.460, 3.449, 3.444, 3.428, 3.428, 3.417, 3.401, 3.390,
        3.379, 3.363, 3.331, 3.299, 3.267, 3.213, 3.149, 3.041, 3.000, 3.000,
        0.000
    };
};

#endif