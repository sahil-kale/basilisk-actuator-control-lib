#include "bridge_hbridge_drv8801.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math_util.hpp"
#include "mock_hal_clock.hpp"
#include "mock_hbridge.hpp"
#include "stepper_control_loop.hpp"

// Make a class that inherits from StepperControlLoop so we can test it
namespace control_loop {

using namespace ::testing;
class StepperControlLoopTest : public control_loop::StepperControlLoop {
   public:
    StepperControlLoopParams default_params{.i_hold = 0.5, .i_run = 1.0, .max_speed = 10.0};

    BrushedControlLoop::BrushedControlLoopParams default_brushed_params{
        .brake_mode = BrushedControlLoop::BrushedBrakeType::COAST,
        .deadtime_us = 0.0f,
        .current_controller_params = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f},
    };

    NiceMock<hwbridge::MOCK_HBRIDGE> bridge_a;
    NiceMock<hwbridge::MOCK_HBRIDGE> bridge_b;
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> clock;

    BrushedControlLoop brushed_control_loop_a{bridge_a, clock};
    BrushedControlLoop brushed_control_loop_b{bridge_b, clock};

    StepperControlLoopTest() : StepperControlLoop(brushed_control_loop_a, brushed_control_loop_b, clock) {}

    void init_brushed_control_loops() {
        brushed_control_loop_a.init(&default_brushed_params);
        brushed_control_loop_b.init(&default_brushed_params);
    }

    // Make the private functions public so we can test them
    using StepperControlLoop::determine_current_setpoints;
};

TEST(stepper_motor_test, test_current_setpoint) {
    StepperControlLoopTest stepper_control_loop_test;
    // Test the determine_current_setpoint_scalars function
    // Test the case where the electrical angle is 0
    auto current_setpoint_scalars = stepper_control_loop_test.determine_current_setpoints(1.0, 0);
    EXPECT_NEAR(current_setpoint_scalars.first, 1.0, math::ACCEPTABLE_FLOAT_ERROR);
    EXPECT_NEAR(current_setpoint_scalars.second, 0.0, math::ACCEPTABLE_FLOAT_ERROR);

    // Test the case where the electrical angle is pi/2
    current_setpoint_scalars = stepper_control_loop_test.determine_current_setpoints(1.0, math::M_PI_FLOAT / 2);
    EXPECT_NEAR(current_setpoint_scalars.first, 0.0, math::ACCEPTABLE_FLOAT_ERROR);
    EXPECT_NEAR(current_setpoint_scalars.second, 1.0, math::ACCEPTABLE_FLOAT_ERROR);

    // Test the case where the electrical angle is pi
    current_setpoint_scalars = stepper_control_loop_test.determine_current_setpoints(1.0, math::M_PI_FLOAT);
    EXPECT_NEAR(current_setpoint_scalars.first, -1.0, math::ACCEPTABLE_FLOAT_ERROR);
    EXPECT_NEAR(current_setpoint_scalars.second, 0.0, math::ACCEPTABLE_FLOAT_ERROR);

    // Test the case where the electrical angle is 3pi/2
    current_setpoint_scalars = stepper_control_loop_test.determine_current_setpoints(1.0, 3 * math::M_PI_FLOAT / 2);
    EXPECT_NEAR(current_setpoint_scalars.first, 0.0, math::ACCEPTABLE_FLOAT_ERROR);
    EXPECT_NEAR(current_setpoint_scalars.second, -1.0, math::ACCEPTABLE_FLOAT_ERROR);
}

// Test that an uninitialized control loop returns an error
TEST(stepper_motor_test, test_uninitialized_control_loop) {
    StepperControlLoopTest stepper_control_loop_test;
    // Run the control loop with a speed of 0
    auto status = stepper_control_loop_test.run(0);
    EXPECT_EQ(status, StepperControlLoop::StepperControlLoopStatus::ControlLoopBaseStatus::ERROR);

    // Poll the StepperControlLoopStatus for the error
    auto stepper_status = stepper_control_loop_test.get_status();
    EXPECT_EQ(stepper_status.get_error(StepperControlLoop::StepperControlLoopStatus::StepperControlLoopError::PARAMS_NOT_SET),
              true);

    // Now, initialize the control loop
    stepper_control_loop_test.init(&stepper_control_loop_test.default_params);

    stepper_control_loop_test.init_brushed_control_loops();

    // Run the control loop with a speed of 0
    status = stepper_control_loop_test.run(0.0f);
    // Poll the StepperControlLoopStatus for the error
    stepper_status = stepper_control_loop_test.get_status();

    // Ensure that the status is OK
    EXPECT_EQ(status, StepperControlLoop::StepperControlLoopStatus::ControlLoopBaseStatus::OK);
}

// Test that on init, the number of steps is reset to 0.0f. Further, test that setting the steps to a non-zero value
// makes us return that value
TEST(stepper_motor_test, reset_and_set_steps) {
    StepperControlLoopTest stepper_control_loop_test;
    // Init the control loop
    stepper_control_loop_test.init(&stepper_control_loop_test.default_params);

    // Expect the number of steps to be 0.0f
    EXPECT_EQ(stepper_control_loop_test.get_steps(), 0.0f);

    // Set the number of steps to 10.0f
    stepper_control_loop_test.set_steps(10.0f);

    // Expect the number of steps to be 10.0f
    EXPECT_EQ(stepper_control_loop_test.get_steps(), 10.0f);
}

}  // namespace control_loop