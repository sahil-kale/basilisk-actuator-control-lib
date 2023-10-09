#include "brushless_foc_control_loop.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math.h"
#include "math_foc.hpp"
#include "mock_bridge_3phase.hpp"
#include "mock_hal_adc.hpp"
#include "mock_hal_clock.hpp"
#include "mock_hal_timer.hpp"
#include "mock_rotor_estimator.hpp"

namespace control_loop {
using namespace ::testing;

basilisk_hal::MOCK_HAL_CLOCK mock_clock;

class BrushlessFocControlLoopTest {
   public:
    // Define a mock HAL clock object

    // Define a mock 3 phase bridge
    hwbridge::MOCK_BRIDGE_3PHASE mock_bridge_3phase_;

    // Define the control loop object
    BrushlessFocControlLoop brushless_foc_control_loop_;

    BrushlessFocControlLoop::BrushlessFocControLoopParams test_params_{
        .kp_q_current = 0.0f,
        .ki_q_current = 0.0f,
        .kd_q_current = 0.0f,
        .kp_d_current = 0.0f,
        .ki_d_current = 0.0f,
        .kd_d_current = 0.0f,
        .foc_start_timeout_period_us = 1000000,
        .speed_to_iq_gain = 0.0f,
        .i_d_reference = 0.0f,
        .open_loop_full_speed_theta_velocity = 0.0f,
        .pwm_control_type = BrushlessFocControlLoop::BrushlessFocPwmControlType::SPACE_VECTOR,
    };

    BrushlessFocControlLoopTest(bldc_rotor_estimator::BldcElectricalRotorPositionEstimator& rotor_position_estimator,
                                basilisk_hal::HAL_CLOCK& clock_)
        : mock_bridge_3phase_(), brushless_foc_control_loop_(mock_bridge_3phase_, clock_, rotor_position_estimator) {}

    BrushlessFocControlLoop::BrushlessFocControlLoopState get_desired_state(
        float motor_speed, const BrushlessFocControlLoop::BrushlessFocControlLoopState current_state) {
        return brushless_foc_control_loop_.get_desired_state(motor_speed, current_state);
    }

    void init(BrushlessFocControlLoop::BrushlessFocControLoopParams* params) { brushless_foc_control_loop_.init(params); }

    void update_state(BrushlessFocControlLoop::BrushlessFocControlLoopState state) { brushless_foc_control_loop_.state_ = state; }

    BrushlessFocControlLoop::BrushlessFocControlLoopType get_control_loop_type(bool is_estimator_valid) {
        return brushless_foc_control_loop_.get_desired_control_loop_type(is_estimator_valid);
    }
};

// Test the state machine transition from stop to start, and a time out makes it go back to stop
TEST(BrushlessFocTest, test_stop_to_start_to_run) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessFocControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure the desired state is NOT_INITIALIZED
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessFocControlLoop::BrushlessFocControlLoopState::NOT_INITIALIZED),
              BrushlessFocControlLoop::BrushlessFocControlLoopState::NOT_INITIALIZED);

    test_control_loop.init(&test_control_loop.test_params_);

    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessFocControlLoop::BrushlessFocControlLoopState::NOT_INITIALIZED),
              BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP),
              BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP);

    // Call the desired state function with a time of foc_start_timeout_period -1, time at start of 0, and a motor speed of 0.1,
    // with the rotor estimator valid Ensure that the desired state is run
    EXPECT_EQ(test_control_loop.get_desired_state(0.1, BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP),
              BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN);
}

// Test the state machine transition from run to stop
TEST(BrushlessFocTest, test_run_to_stop) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessFocControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN),
              BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP);
}

}  // namespace control_loop