#ifndef BRUSHED_CONTROL_LOOP_TEST_HPP
#define BRUSHED_CONTROL_LOOP_TEST_HPP
#include "brushed_control_loop.hpp"

#include "bridge_hbridge.hpp"
#include "gmock/gmock.h"
#include "mock_hal_clock.hpp"
#include "mock_hbridge.hpp"

namespace control_loop {
// Make a class to extend the BrushedControlLoop class so we can access its protected members
class BrushedControlLoopTest : public BrushedControlLoop {
   public:
    BrushedControlLoopParams default_params{
        .brake_mode = BrushedBrakeType::COAST,
        .deadtime_us = 0,
        .current_controller_params =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
            },
    };

    hwbridge::MOCK_HBRIDGE mock_bridge_;
    basilisk_hal::MOCK_HAL_CLOCK mock_clock_;

    BrushedControlLoopTest() : BrushedControlLoop(mock_bridge_, mock_clock_) {}

    using BrushedControlLoop::get_desired_state;
    using BrushedControlLoop::run_state;
    using BrushedControlLoop::state_;
};

using namespace ::testing;

TEST(BrushedControlLoopTest, test_stop_to_run) {
    // Test that a control loop in the STOP state can transition to the RUN state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(
        control_loop.get_desired_state(0.1, 0.0, BrushedControlLoop::BrushedControlLoopState::STOP, 0, deadband_start_time, 0),
        BrushedControlLoop::BrushedControlLoopState::RUN);
}

// Test RUN to STOP
TEST(BrushedControlLoopTest, test_run_to_stop) {
    // Test that a control loop in the RUN state can transition to the STOP state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(
        control_loop.get_desired_state(0.0, 0.1, BrushedControlLoop::BrushedControlLoopState::RUN, 0, deadband_start_time, 0),
        BrushedControlLoop::BrushedControlLoopState::STOP);
}

// Test that a RUN from a speed of 0.1 -> -0.1 does NOT transition to DEADTIME_PAUSE if the deadtime_pause_time_us is 0
TEST(BrushedControlLoopTest, test_run_to_run_without_deadtime) {
    // Test that a control loop in the RUN state can transition to the DEADTIME_PAUSE state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(
        control_loop.get_desired_state(-0.1, 0.1, BrushedControlLoop::BrushedControlLoopState::RUN, 0, deadband_start_time, 0),
        BrushedControlLoop::BrushedControlLoopState::RUN);
}

// Test that a RUN from a speed of 0.1 -> -0.1 DOES transition to DEADTIME_PAUSE if the deadtime_pause_time_us is 1
TEST(BrushedControlLoopTest, test_run_to_deadtime_pause_with_deadtime) {
    // Test that a control loop in the RUN state can transition to the DEADTIME_PAUSE state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(
        control_loop.get_desired_state(-0.1, 0.1, BrushedControlLoop::BrushedControlLoopState::RUN, 0, deadband_start_time, 1),
        BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE);
}

// Test that we can transition from DEADTIME_PAUSE to RUN if the deadtime has expired
TEST(BrushedControlLoopTest, test_deadtime_pause_to_run) {
    // Test that a control loop in the RUN state can transition to the DEADTIME_PAUSE state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(control_loop.get_desired_state(-0.1, 0.1, BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE, 0,
                                             deadband_start_time, 1),
              BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE);
    EXPECT_EQ(control_loop.get_desired_state(-0.1, 0.1, BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE, 1,
                                             deadband_start_time, 1),
              BrushedControlLoop::BrushedControlLoopState::RUN);
}

// Test that we can transition from DEADTIME_PAUSE to STOP if the speed is 0
TEST(BrushedControlLoopTest, test_deadtime_pause_to_stop) {
    // Test that a control loop in the RUN state can transition to the DEADTIME_PAUSE state if the speed is 0
    BrushedControlLoopTest control_loop;
    control_loop.init(&control_loop.default_params);
    utime_t deadband_start_time = 0;
    EXPECT_EQ(control_loop.get_desired_state(0.0, 0.1, BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE, 0,
                                             deadband_start_time, 1),
              BrushedControlLoop::BrushedControlLoopState::STOP);
}

// Test the run state function with a speed of 0, and a brake mode of COAST, then BRAKE_LOW_SIDE, then BRAKE_HIGH_SIDE
TEST(BrushedControlLoopTest, test_run_state_stop) {
    BrushedControlLoopTest control_loop;
    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode = BrushedControlLoop::BrushedBrakeType::COAST,
        .deadtime_us = 0,
        .current_controller_params =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
            },
    };

    control_loop.init(&params);
    control_loop.state_ = BrushedControlLoop::BrushedControlLoopState::STOP;

    hwbridge::HBridge::HBridgeInput expected_output{
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = false,
    };

    // Get the actual output
    hwbridge::HBridge::HBridgeInput actual_output = control_loop.run_state(0.0f, control_loop.state_);

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);

    // Set the params to BRAKE_LOW_SIDE
    params.brake_mode = BrushedControlLoop::BrushedBrakeType::BRAKE_LOW_SIDE;

    // Reinitialize the control loop
    control_loop.init(&params);

    // Get the actual output
    actual_output = control_loop.run_state(0.0f, control_loop.state_);

    // Set the expected output low side gpio's to true
    expected_output.low_side_a_gpio_state = true;
    expected_output.low_side_b_gpio_state = true;

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);

    // Set the params to BRAKE_HIGH_SIDE
    params.brake_mode = BrushedControlLoop::BrushedBrakeType::BRAKE_HIGH_SIDE;

    // Reinitialize the control loop
    control_loop.init(&params);

    // Get the actual output
    actual_output = control_loop.run_state(0.0f, control_loop.state_);

    // Set the expected output high side duty cycles to 1.0, and the low side gpio's to false
    expected_output.duty_cycle_a_h = 1.0f;
    expected_output.duty_cycle_b_h = 1.0f;
    expected_output.low_side_a_gpio_state = false;
    expected_output.low_side_b_gpio_state = false;

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);
}

// Test the run state function with a state of DEADTIME_PAUSE and a speed of -0.3
// The bridge should just be floating
TEST(BrushedControlLoopTest, test_run_state_deadtime_pause) {
    BrushedControlLoopTest control_loop;
    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode = BrushedControlLoop::BrushedBrakeType::BRAKE_HIGH_SIDE,
        .deadtime_us = 0,
        .current_controller_params =
            {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
            },
    };

    control_loop.init(&params);
    control_loop.state_ = BrushedControlLoop::BrushedControlLoopState::DEADTIME_PAUSE;

    hwbridge::HBridge::HBridgeInput expected_output{
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = false,
    };

    // Get the actual output
    hwbridge::HBridge::HBridgeInput actual_output = control_loop.run_state(0.0f, control_loop.state_);

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);
}

// Test the run state function with a state of RUN and a speed of 0.3.
// Expect the A duty cycle to be 0.3, and the B duty cycle to be 0.0
// Expect A low side gpio to be false, and B low side gpio to be true
TEST(BrushedControlLoopTest, test_run_positive_dir) {
    BrushedControlLoopTest control_loop;

    control_loop.init(&control_loop.default_params);
    control_loop.state_ = BrushedControlLoop::BrushedControlLoopState::RUN;

    hwbridge::HBridge::HBridgeInput expected_output{
        .duty_cycle_a_h = 0.3f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = true,
    };

    // Get the actual output
    hwbridge::HBridge::HBridgeInput actual_output = control_loop.run_state(0.3f, control_loop.state_);

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);
}

// Test the run state function with a state of RUN and a speed of -0.3.
// Expect the A duty cycle to be 0.0, and the B duty cycle to be 0.3
// Expect A low side gpio to be true, and B low side gpio to be false
TEST(BrushedControlLoopTest, test_run_negative_dir) {
    BrushedControlLoopTest control_loop;

    control_loop.init(&control_loop.default_params);
    control_loop.state_ = BrushedControlLoop::BrushedControlLoopState::RUN;

    hwbridge::HBridge::HBridgeInput expected_output{
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = true,
        .duty_cycle_b_h = 0.3f,
        .low_side_b_gpio_state = false,
    };

    // Get the actual output
    hwbridge::HBridge::HBridgeInput actual_output = control_loop.run_state(-0.3f, control_loop.state_);

    // Compare the actual output to the expected output
    EXPECT_EQ(actual_output.duty_cycle_a_h, expected_output.duty_cycle_a_h);
    EXPECT_EQ(actual_output.low_side_a_gpio_state, expected_output.low_side_a_gpio_state);
    EXPECT_EQ(actual_output.duty_cycle_b_h, expected_output.duty_cycle_b_h);
    EXPECT_EQ(actual_output.low_side_b_gpio_state, expected_output.low_side_b_gpio_state);
}

// Do an E2E test of the control loop where we run the control loop at a speed of 0.3
// Expect the mock bridge to be called with the appropriate duty cycles and gpio states
// Then, run the control loop at a speed of -0.6
// Expect a dead time pause to occur
// After dead time, expect the mock bridge to be called with the appropriate duty cycles and gpio states
// Then, run the control loop at a speed of 0.0
// Expect the mock bridge to be called with the appropriate duty cycles and gpio states of the brake mode
TEST(BrushedControlLoopTest, test_pos_to_neg_then_stop) {
    BrushedControlLoopTest control_loop;
    BrushedControlLoop::BrushedControlLoopParams params{.brake_mode = BrushedControlLoop::BrushedBrakeType::BRAKE_LOW_SIDE,
                                                        .deadtime_us = 60,
                                                        .current_controller_params = {
                                                            .kp = 0.0f,
                                                            .ki = 0.0f,
                                                            .kd = 0.0f,
                                                        }};

    control_loop.init(&params);
    // Set the mock time to 1
    EXPECT_CALL(control_loop.mock_clock_, get_time_us()).WillOnce(Return(1));

    // Set the mock bridge expectations
    hwbridge::HBridge::HBridgeInput expected_output = {
        .duty_cycle_a_h = 0.3f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = true,
    };
    EXPECT_CALL(control_loop.mock_bridge_, run(expected_output)).WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(0.3f);

    // Set the mock time to 20
    EXPECT_CALL(control_loop.mock_clock_, get_time_us()).WillOnce(Return(20));

    // Set the mock bridge expectations
    expected_output = {
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = false,
    };

    EXPECT_CALL(control_loop.mock_bridge_, run(expected_output)).WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(-0.6f);

    // Set the mock bridge expectations to be floating until 20 + params.deadtime_us. In this case, set the mock time to that
    // value - 1
    EXPECT_CALL(control_loop.mock_clock_, get_time_us()).WillOnce(Return(20 + params.deadtime_us - 1));

    // Set the mock bridge expectations
    expected_output = {
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = false,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = false,
    };
    EXPECT_CALL(control_loop.mock_bridge_, run(expected_output)).WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(-0.6f);

    // Now, set the mock time to 20 + params.deadtime_us
    EXPECT_CALL(control_loop.mock_clock_, get_time_us()).WillOnce(Return(20 + params.deadtime_us));

    // Set the mock bridge expectations
    expected_output = {
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = true,
        .duty_cycle_b_h = 0.6f,
        .low_side_b_gpio_state = false,
    };

    EXPECT_CALL(control_loop.mock_bridge_, run(expected_output)).WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(-expected_output.duty_cycle_b_h);

    // Set the mock time to 1000
    EXPECT_CALL(control_loop.mock_clock_, get_time_us()).WillOnce(Return(1000));

    // Set the mock bridge expectations
    expected_output = {
        .duty_cycle_a_h = 0.0f,
        .low_side_a_gpio_state = true,
        .duty_cycle_b_h = 0.0f,
        .low_side_b_gpio_state = true,
    };
    EXPECT_CALL(control_loop.mock_bridge_, run(expected_output)).WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(0.0f);
}

}  // namespace control_loop

#endif  // BRUSHED_CONTROL_LOOP_TEST_HPP