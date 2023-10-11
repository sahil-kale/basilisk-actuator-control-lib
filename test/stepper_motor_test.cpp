#include "bridge_hbridge_drv8801.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math_util.hpp"
#include "mock_hal_clock.hpp"
#include "mock_hbridge.hpp"
#include "stepper_control_loop.hpp"

// Make a class that inherits from StepperControlLoop so we can test it
namespace control_loop {
class StepperControlLoopTest : public control_loop::StepperControlLoop {
   public:
    StepperControlLoopParams default_params{.i_hold = 0.5, .i_run = 1.0, .max_speed = 10.0};

    hwbridge::MOCK_HBRIDGE bridge_a;
    hwbridge::MOCK_HBRIDGE bridge_b;
    basilisk_hal::MOCK_HAL_CLOCK clock;

    BrushedControlLoop brushed_control_loop_a{bridge_a, clock};
    BrushedControlLoop brushed_control_loop_b{bridge_b, clock};

    StepperControlLoopTest() : StepperControlLoop(brushed_control_loop_a, brushed_control_loop_b, clock) {}
    // Make the private functions public so we can test them
    using StepperControlLoop::determine_current_setpoints;

    // Make the protected variables public so we can test them
    using StepperControlLoop::electrical_angle_;
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

}  // namespace control_loop