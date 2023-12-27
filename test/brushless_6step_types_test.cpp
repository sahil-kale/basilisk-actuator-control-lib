#include "brushless_6step_commutation.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math_util.hpp"

using namespace ::testing;

namespace control_loop {

void test_commutation_step(const Bldc6Step::commutation_step_t& commutation_step, uint8_t expected_sector) {
    EXPECT_EQ(commutation_step.signals[0], Bldc6Step::commutation_steps[expected_sector].signals[0]);
    EXPECT_EQ(commutation_step.signals[1], Bldc6Step::commutation_steps[expected_sector].signals[1]);
    EXPECT_EQ(commutation_step.signals[2], Bldc6Step::commutation_steps[expected_sector].signals[2]);
}

// Test the angle to commutation step function
TEST(Bldc6StepCommutationTest, test_angle_to_commutation) {
    // Get the commutation step from the angle 0 radians
    auto commutation_step = Bldc6Step::determine_commutation_step_from_theta(0.0f);
    test_commutation_step(commutation_step, 0);

    // Get the commutation step from the angle 1/12th of a revolution
    commutation_step = Bldc6Step::determine_commutation_step_from_theta(math::M_PI_FLOAT / 6.0f);
    test_commutation_step(commutation_step, 0);

    // Get the angle for 45 degrees
    commutation_step = Bldc6Step::determine_commutation_step_from_theta(math::M_PI_FLOAT / 4.0f);
    test_commutation_step(commutation_step, 1);

    // Get pi radians
    commutation_step = Bldc6Step::determine_commutation_step_from_theta(math::M_PI_FLOAT);
    test_commutation_step(commutation_step, 3);
}
}  // namespace control_loop