#include "brushless_6step_commutation.hpp"
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

// Test the generate commutation duty cycle function with the commutation step and commutation signal lookup
TEST(Bldc6StepCommutationTest, test_6_step_duty_cycle) {
    // Phase command struct
    hwbridge::Bridge3Phase::phase_command_t phase_command[3];

    for (uint8_t i = 0; i < Bldc6Step::num_commutation_steps; i++) {
        // Check that the phase command is correct for a motor speed of 1
        determine_inverter_duty_cycles_trap(phase_command, Bldc6Step::commutation_steps[i], 1.0f);
        // Grab the expected commutation signal
        auto expected_commutation_signal = Bldc6Step::commutation_steps[i];
        // Check the phase command for each phase
        for (uint8_t j = 0; j < hwbridge::Bridge3Phase::NUM_PHASES; j++) {
            // Check the high signal duty cycle
            if (expected_commutation_signal.signals[j] == Bldc6Step::CommutationSignal::HIGH) {
                // Expect the duty cycle to be 1
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, 1.0f);
                // Expect the low side to be inverted
                EXPECT_FLOAT_EQ(phase_command[j].invert_low_side, true);
            }

            // Check the low signal duty cycle
            if (expected_commutation_signal.signals[j] == Bldc6Step::CommutationSignal::LOW) {
                // Expect the duty cycle to be 0
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, 0.0f);
                // Expect the low side to be inverted
                EXPECT_FLOAT_EQ(phase_command[j].invert_low_side, true);
            }

            const bool is_high_z = (expected_commutation_signal.signals[j] == Bldc6Step::CommutationSignal::Z_FALLING) ||
                                   (expected_commutation_signal.signals[j] == Bldc6Step::CommutationSignal::Z_RISING);
            if (is_high_z) {
                // Expect the duty cycle to be 0
                EXPECT_FLOAT_EQ(phase_command[j].duty_cycle_high_side, 0.0f);
                // Expect the low side to be inverted
                EXPECT_FLOAT_EQ(phase_command[j].invert_low_side, false);
            }
        }
    }
}
}  // namespace control_loop