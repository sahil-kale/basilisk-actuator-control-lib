#ifndef BRUSHED_CONTROL_LOOP_TEST_HPP
#define BRUSHED_CONTROL_LOOP_TEST_HPP
#include "brushed_control_loop.hpp"

#include "gmock/gmock.h"

namespace control_loop {

using namespace ::testing;

class BrushedControlLoopTest {
   public:
    BrushedControlLoopTest() : brushed_control_loop_() {}

    // Define a getter method for the brushed_control_loop_ member variable that returns a reference
    BrushedControlLoop& get_brushed_control_loop() { return brushed_control_loop_; }

    // Define an accessor method for the compute_motor_speed_outputs method
    BrushedControlLoop::h_bridge_motor_speed_outputs_t compute_motor_speed_outputs(float motor_speed, bool brake_mode,
                                                                                   utime_t current_time_us) {
        return brushed_control_loop_.compute_motor_speed_outputs(motor_speed, brake_mode, current_time_us);
    }

    utime_t& get_last_speed_dir_change_time_us() { return brushed_control_loop_.last_speed_dir_change_time_us_; }

    void init(BrushedControlLoop::BrushedControlLoopParams* params) { brushed_control_loop_.init(params); }

   private:
    BrushedControlLoop brushed_control_loop_;
};

// Test the compute_motor_speed_outputs method
// Test that a motor speed of 0.0f with brake mode false returns a duty cycle of 0.0f for all pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_zero_speed_no_brake) {
    BrushedControlLoopTest brushed_control_loop;
    control_loop::BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = false,
        .h_bridge_dead_time_us = 0,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs =
        brushed_control_loop.compute_motor_speed_outputs(0.0f, false, params.h_bridge_dead_time_us);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.0f);
}

// Test that a motor speed of 0.0f with brake mode true returns a duty cycle of 1.0f for low pins and 0.0f for high pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_zero_speed_brake) {
    BrushedControlLoopTest brushed_control_loop;

    control_loop::BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = true,
        .h_bridge_dead_time_us = 0,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs =
        brushed_control_loop.compute_motor_speed_outputs(0.0f, true, params.h_bridge_dead_time_us);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 1.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 1.0f);
}

// Test that a motor speed of 1.0f returns a duty cycle of 1.0f A high and B low and 0.0f for the other pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_full_speed) {
    BrushedControlLoopTest brushed_control_loop;

    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = false,
        .h_bridge_dead_time_us = 0,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs = brushed_control_loop.compute_motor_speed_outputs(
        control_loop::ControlLoop::MAX_MOTOR_SPEED, false, params.h_bridge_dead_time_us);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 1.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 1.0f);
}

// Test that a motor speed of -1.0f returns a duty cycle of 1.0f A low and B high and 0.0f for the other pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_full_reverse_speed) {
    BrushedControlLoopTest brushed_control_loop;

    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = false,
        .h_bridge_dead_time_us = 0,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs = brushed_control_loop.compute_motor_speed_outputs(
        -control_loop::ControlLoop::MAX_MOTOR_SPEED, false, params.h_bridge_dead_time_us);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 1.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 1.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.0f);
}

// Test that a motor speed of 0.5f returns a duty cycle of 0.5f A high and B low and 0.0f for the other pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_half_speed) {
    BrushedControlLoopTest brushed_control_loop;

    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = false,
        .h_bridge_dead_time_us = 0,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs = brushed_control_loop.compute_motor_speed_outputs(
        control_loop::ControlLoop::MAX_MOTOR_SPEED * 0.5f, false, params.h_bridge_dead_time_us);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.5f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.5f);
}

// Test the dead time by commanding -0.5f and then 0.5f and checking that the duty cycle is 0.0f for pins
TEST(BrushedControlLoopTests, compute_motor_speed_outputs_dead_time) {
    utime_t test_time = 100000;
    BrushedControlLoopTest brushed_control_loop;

    BrushedControlLoop::BrushedControlLoopParams params{
        .brake_mode_enabled = false,
        .h_bridge_dead_time_us = 10,
    };

    brushed_control_loop.init(&params);

    BrushedControlLoop::h_bridge_motor_speed_outputs_t motor_speed_outputs =
        brushed_control_loop.compute_motor_speed_outputs(-0.5f * control_loop::ControlLoop::MAX_MOTOR_SPEED, false, test_time);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.5f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.5f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.0f);
    motor_speed_outputs =
        brushed_control_loop.compute_motor_speed_outputs(0.5f * control_loop::ControlLoop::MAX_MOTOR_SPEED, false, test_time + 1);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.0f);
    motor_speed_outputs = brushed_control_loop.compute_motor_speed_outputs(control_loop::ControlLoop::MAX_MOTOR_SPEED * 0.5f,
                                                                           false, params.h_bridge_dead_time_us + test_time + 1);
    EXPECT_EQ(motor_speed_outputs.DC_A_HIGH, 0.5f);
    EXPECT_EQ(motor_speed_outputs.DC_A_LOW, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_HIGH, 0.0f);
    EXPECT_EQ(motor_speed_outputs.DC_B_LOW, 0.5f);
}

}  // namespace control_loop

#endif  // BRUSHED_CONTROL_LOOP_TEST_HPP