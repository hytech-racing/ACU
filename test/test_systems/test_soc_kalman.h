#include "gtest/gtest.h"
#include <cmath>
#include "SOCKalmanFilter.h"

float coulomb_count(float initial_soc, float current, float dt) {
    return initial_soc - (current * dt / soc_ekf_constants::CAPACITY_AS);
}

TEST(SOCKalmanFilterTesting, initialization_from_voltage) {
    SOCKalmanFilter ekf;
    ekf.init(3.972f);
    float soc = ekf.get_soc();
    EXPECT_NEAR(soc, 1.0f, 0.01f);

    SOCKalmanFilter ekf2;
    ekf2.init(3.56f);
    float soc2 = ekf2.get_soc();
    EXPECT_NEAR(soc2, 0.5f, 0.1f);

    SOCKalmanFilter ekf3;
    ekf3.init(3.0f);
    float soc3 = ekf3.get_soc();
    EXPECT_NEAR(soc3, 0.0f, 0.01f);
}

TEST(SOCKalmanFilterTesting, v1_starts_at_0) {
    SOCKalmanFilter ekf;
    ekf.init(3.7f);
    EKFState_s state = ekf.get_state();
    EXPECT_NEAR(state.v1, 0.0f, 0.01f);
}

TEST(SOCKalmanFilterTesting, coulomb_counting_accuracy) {
    SOCKalmanFilter ekf;
    ekf.init(3.7f);
    float start_soc = ekf.get_soc();

    float current = 10.0f;
    float dt = 100.0f;
    float start_ocv = 3.7f;

    float ir_drop = current * soc_ekf_constants::R0; 
    float input_voltage = start_ocv - ir_drop;

    ekf.update(current, input_voltage, dt);

    float expected_delta = (current * dt) / soc_ekf_constants::CAPACITY_AS;

    EXPECT_NEAR(ekf.get_soc(), start_soc - expected_delta, 0.01f);
}

TEST(SOCKalmanFilterTesting, v1_dynamics_time_constant) {
    SOCKalmanFilter ekf;
    float start_voltage = 3.7f;
    ekf.init(start_voltage); 

    float current = 100.0f;
    float dt = soc_ekf_constants::TIME_CONSTANT;
    float max_v1_possible = current * soc_ekf_constants::R1;
    float expected_v1 = max_v1_possible * 0.6321f;

    float ir_drop = current * soc_ekf_constants::R0;
    float input_voltage = start_voltage - ir_drop - expected_v1;

    

    EKFState_s state = ekf.update(current, input_voltage, dt);

    EXPECT_NEAR(state.v1, expected_v1, 0.05f);
}

TEST(SOCKalmanFilterTesting, convergence_correction_zero_current) {
    SOCKalmanFilter ekf;
    
    // 1. Initialize at ~50% SoC (3.56V)
    // Physics Model says: "If current is 0, SoC stays at 50% forever."
    float start_voltage = 3.4f; 
    ekf.init(start_voltage);
    float start_soc = ekf.get_soc();

    // 2. The Conflict
    // Sensor says: "Actually, voltage is 3.8V (approx 80-90% SoC)."
    // The Filter MUST override the Physics Model and climb up.
    float target_voltage = 3.7f; 
    float current = 0.0f;
    float dt = 0.1f;

    // 3. Run the Correction Loop
    for (int i = 0; i < 3000; i++) {
        ekf.update(current, target_voltage, dt);
    }

    // 4. Verification
    // SoC must have climbed significantly from start (Correction working)
    EXPECT_GT(ekf.get_soc(), start_soc + 0.15f);
    
    // Robustness Check: It shouldn't have jumped instantly to 100% or NaN.
    // It should be approaching the target (approx 0.9) but maybe not fully there yet 
    // (depending on Q/R tuning).
    EXPECT_LT(ekf.get_soc(), 1.0f); 
}

TEST(SOCKalmanFilterTesting, safety_clamping_bounds) {
    SOCKalmanFilter ekf;
    
    ekf.init(4.2f);
    ekf.update(-100.0f, 4.5f, 3600.0f); 
    EXPECT_LE(ekf.get_soc(), 1.0f);

    ekf.init(3.0f);
    ekf.update(100.0f, 2.0f, 3600.0f); 
    EXPECT_GE(ekf.get_soc(), 0.0f);
}

TEST(SOCKalmanFilterTesting, initialization_accuracy) {
    SOCKalmanFilter ekf;
    
    ekf.init(3.972f);
    EXPECT_NEAR(ekf.get_soc(), 1.0f, 0.01f);

    ekf.init(3.0f);
    EXPECT_NEAR(ekf.get_soc(), 0.0f, 0.01f);

    ekf.init(3.56f);
    EXPECT_NEAR(ekf.get_soc(), 0.5f, 0.05f);
}