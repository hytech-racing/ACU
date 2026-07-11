#ifndef SOC_KALMAN_FILTER_H
#define SOC_KALMAN_FILTER_H

/* ETL Library */
#include "etl/singleton.h"

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include <array>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>


namespace soc_ekf_constants
{
    constexpr const float CAPACITY_AS = 49549.0f; // Beginning-of-life measured capacity in Amp-seconds (13.764 Ah * 3600 s/h)
    constexpr const float TOTAL_USABLE_WH_PER_CELL_3C = 44.781f; // Total usable energy per cell from 3C track-pace discharge
    constexpr const float R0 = 0.00184f; // Ohmic resistance (instantaneous voltage drop)
    constexpr const float R1 = 0.0006535f; // Polarization resistance to see slow voltage response
    constexpr const float TIME_CONSTANT = 7.81f; // Relaxation time constant
    constexpr const float C1 = TIME_CONSTANT / R1; // Polarization capacitance

    // EKF tuning parameters (update these to tune the EKF to track SoC better)
    constexpr const float Q_SOC = 1e-6f; // process noise for SoC
    constexpr const float Q_V1 = 1e-6f; // process noise for V1
    constexpr const float R_V1 = 0.1f; // measurement noise for V1

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
     * @param current_soh // current state of health
     */
    EKFState_s update(float current, float voltage, float dt, bool voltage_is_fresh, float current_soh);

    /**
     * @brief Get the soc object
     * @return float state of charge
     */
    float get_soc() const { return _state.soc; }

    /**
     * @brief Get the voltage object
     * @return float voltage
     */
    float get_voltage() const { return _state.v1; }

    /**
     * @brief Get the State of Energy (SoE) as a percentage of usable energy remaining
     * @return float SoE percentage (0.0 to 100.0)
     */
    float get_soe_percentage() const;

    /**
     * @brief Get current state
     * @return Complete state vector
     */
    EKFState_s get_state() const { return _state; }

    /**
     * @brief Reset SoC estimate
     * @param new_soc New SoC value
     * @post SoC updated, uncertainty increased
     */
    void reset_soc(float new_soc);

    // OCV Lookup Table
    static constexpr float VOLTAGE_LOOKUP_TABLE[101] = {
        4.188, 4.175, 4.163, 4.150, 4.138, 4.125, 4.112, 4.104, 4.094, 4.083,
        4.072, 4.061, 4.050, 4.041, 4.033, 4.024, 4.015, 4.005, 3.996, 3.987,
        3.980, 3.973, 3.966, 3.960, 3.955, 3.949, 3.944, 3.938, 3.932, 3.927,
        3.921, 3.915, 3.910, 3.904, 3.898, 3.892, 3.887, 3.881, 3.875, 3.869,
        3.864, 3.858, 3.853, 3.849, 3.844, 3.840, 3.836, 3.832, 3.828, 3.824,
        3.821, 3.818, 3.815, 3.812, 3.809, 3.807, 3.804, 3.802, 3.800, 3.797,
        3.795, 3.793, 3.792, 3.790, 3.789, 3.787, 3.786, 3.785, 3.784, 3.782,
        3.780, 3.778, 3.776, 3.773, 3.770, 3.767, 3.763, 3.759, 3.755, 3.751,
        3.747, 3.742, 3.737, 3.731, 3.725, 3.720, 3.715, 3.710, 3.705, 3.702,
        3.698, 3.692, 3.683, 3.667, 3.642, 3.605, 3.553, 3.474, 3.326, 3.265,
        3.184
    };

    // SOE Lookup Table, usable energy percentage as a function of SoC (3C test)
    static constexpr float ENERGY_LOOKUP_TABLE[101] = {
        0.0, 0.8, 1.7, 2.6, 3.6, 4.5, 5.5, 6.4, 7.4, 8.4,
        9.3, 10.3, 11.3, 12.2, 13.2, 14.2, 15.2, 16.1, 17.1, 18.1,
        19.1, 20.1, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 26.9, 27.9,
        28.9, 29.9, 30.9, 31.9, 32.9, 33.9, 34.8, 35.8, 36.8, 37.8,
        38.8, 39.8, 40.8, 41.8, 42.8, 43.8, 44.8, 45.8, 46.8, 47.8,
        48.7, 49.7, 50.7, 51.7, 52.7, 53.7, 54.7, 55.7, 56.7, 57.8,
        58.8, 59.8, 60.8, 61.8, 62.8, 63.8, 64.8, 65.8, 66.8, 67.8,
        68.8, 69.9, 70.9, 71.9, 72.9, 73.9, 75.0, 76.0, 77.0, 78.0,
        79.1, 80.1, 81.1, 82.2, 83.2, 84.2, 85.3, 86.3, 87.3, 88.4,
        89.4, 90.5, 91.5, 92.6, 93.6, 94.7, 95.7, 96.8, 97.9, 98.9,
        100.0
    };

private:

    EKFState_s _state; // The system state (SoC, V_polarization)

    // Covariance Matrix P (2x2)
    // Tracks the uncertainty of our estimate.
    // P[0][0] = var(SoC), P[1][1] = var(V1)
    float _PMatrix[2][2];

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

};

#endif