#include "6step_util.hpp"
#include "gtest/gtest.h"
#include "math_util.hpp"

using namespace ::testing;

namespace control_loop {
namespace Bldc6Step {

void test_commutation_step(const commutation_step_t& commutation_step, uint8_t expected_sector) {
    EXPECT_EQ(commutation_step.signals[0], commutation_steps[expected_sector].signals[0]);
    EXPECT_EQ(commutation_step.signals[1], commutation_steps[expected_sector].signals[1]);
    EXPECT_EQ(commutation_step.signals[2], commutation_steps[expected_sector].signals[2]);
}

// Test the angle to commutation step function
TEST(Bldc6StepUtils, test_angle_to_commutation) {
    // Get the commutation step from the angle 0 radians
    auto commutation_step = determine_commutation_step_from_theta(0.0f);
    test_commutation_step(commutation_step, 0);

    // Get the commutation step from the angle 1/12th of a revolution
    commutation_step = determine_commutation_step_from_theta(math::M_PI_FLOAT / 6.0f);
    test_commutation_step(commutation_step, 0);

    // Get the angle for 45 degrees
    commutation_step = determine_commutation_step_from_theta(math::M_PI_FLOAT / 4.0f);
    test_commutation_step(commutation_step, 1);

    // Get pi radians
    commutation_step = determine_commutation_step_from_theta(math::M_PI_FLOAT);
    test_commutation_step(commutation_step, 3);
}

// Test the generate commutation duty cycle function with the commutation step and commutation signal lookup
TEST(Bldc6StepUtils, test_6_step_duty_cycle) {
    // Phase command struct
    hwbridge::Bridge3Phase::phase_command_t phase_command[3];
    const float speed = 0.5f;
    // Because we contract the phase command as a complementary switching pair, '0.5' is actually 0V phase-to-neutral
    // As a result, we map the motor speed to the duty cycle as follows:
    // 0.0f -> 0.5f
    // -1.0f -> 0.0f
    // 1.0f -> 1.0f
    const float expected_high_side_duty_cycle = (speed + 1.0f) / 2.0f;

    for (uint8_t i = 0; i < num_commutation_steps; i++) {
        // Check that the phase command is correct for a motor speed of 1
        determine_inverter_duty_cycles_trap(phase_command, commutation_steps[i], speed);
        // Grab the expected commutation signal
        auto expected_commutation_signal = commutation_steps[i];
        // Check the phase command for each phase
        for (uint8_t j = 0; j < hwbridge::Bridge3Phase::NUM_PHASES; j++) {
            // Check the high signal duty cycle
            if (expected_commutation_signal.signals[j] == CommutationSignal::HIGH) {
                // Expect the duty cycle to be 1
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, expected_high_side_duty_cycle);
                // Expect the low side to be enabled
                EXPECT_FLOAT_EQ(phase_command[j].enable, true);
            }

            // Check the low signal duty cycle
            if (expected_commutation_signal.signals[j] == CommutationSignal::LOW) {
                // Expect the duty cycle to be 0
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, 0.0f);
                // Expect the low side to be enabled
                EXPECT_FLOAT_EQ(phase_command[j].enable, true);
            }

            const bool is_high_z = (expected_commutation_signal.signals[j] == CommutationSignal::Z_FALLING) ||
                                   (expected_commutation_signal.signals[j] == CommutationSignal::Z_RISING);
            if (is_high_z) {
                // Expect the duty cycle to be 0
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, 0.0f);
                // Expect the low side to be enabled
                EXPECT_FLOAT_EQ(phase_command[j].enable, false);
            }
        }
    }
}

}  // namespace Bldc6Step
}  // namespace control_loop