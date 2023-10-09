#include "bridge_hbridge_drv8801.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"
#include "stepper_control_loop.hpp"

namespace control_loop {
// Test the determineElectricalAngle function. For now, this will use simple 4 block commutation

// Assert that a desired speed of 0 returns the same previous angle
TEST(StepperControlLoop, determineElectricalAngleZeroSpeed) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    // Assert that the angle is the same as the previous angle
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(0.0f, 0.0f, 0.0f), 0.0f);
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(0.0f, 0.0f, 90.0f), 90.0f);
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(0.0f, 0.0f, 180.0f), 180.0f);
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(0.0f, 0.0f, 270.0f), 270.0f);
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(0.0f, 0.0f, 360.0f), 360.0f);
}

// Assert that at a desired speed's time delta, the angle is 90 degrees greater than previous angle
TEST(StepperControlLoop, determineElectricalAngleMaxSpeed) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    const float test_max_speed_steps_per_second = 200.0f;
    // Determine the time delta to be at max speed
    const float test_time_delta = 1.0f / test_max_speed_steps_per_second;
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, test_max_speed_steps_per_second, 0.0f), 90.0f);
    // make an angle 1 degree. Expect the angle to be 271 degrees
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, -test_max_speed_steps_per_second, 1.0f), 271.0f);
    // make an angle 359 degrees. Expect the angle to be 89 degrees
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, test_max_speed_steps_per_second, 359.0f), 89.0f);
}

// Microstepping test: assert that at half a desired's speed's time delta, the angle is 45 degrees greater than previous angle
TEST(StepperControlLoop, determineElectricalAngleHalfSpeed) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    const float test_max_speed_steps_per_second = 200.0f;
    // Determine the time delta to be at max speed
    const float test_time_delta = 1.0f / (2.0f * test_max_speed_steps_per_second);
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, test_max_speed_steps_per_second, 0.0f), 45.0f);
    // make an angle 1 degree. Expect the angle to be 316 degrees
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, -test_max_speed_steps_per_second, 1.0f), 316.0f);
    // make an angle 359 degrees. Expect the angle to be 44 degrees
    EXPECT_EQ(stepper_control_loop.determineElectricalAngle(test_time_delta, test_max_speed_steps_per_second, 359.0f), 44.0f);
}

// Test the motor current value calculation function
// Assert that a desired angle of 0 makes the A current scaler 1 and B current scaler 0
TEST(StepperControlLoop, determineMotorCurrentValuesAngle0) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    // Assert that the angle is the same as the previous angle
    EXPECT_EQ(stepper_control_loop.determineCurrentSetpointScalars(0.0f), std::make_pair(1.0f, 0.0f));
}

// Assert that a desired angle of 90 makes the A current scaler 0 and B current scaler 1
TEST(StepperControlLoop, determineMotorCurrentValuesAngle90) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    // Assert that the angle is the same as the previous angle
    std::pair<float, float> current_scalars = stepper_control_loop.determineCurrentSetpointScalars(90.0f);
    EXPECT_NEAR(current_scalars.first, 0.0f, 0.01);
    EXPECT_NEAR(current_scalars.second, 1.0f, 0.01);
}

// Assert that a desired angle of 315 makes the A current scaler 1/sqrt(2) and B current scaler -1/sqrt(2)
TEST(StepperControlLoop, determineMotorCurrentValuesAngle315) {
    // Create a mock clock
    basilisk_hal::MOCK_HAL_CLOCK clock;
    // Create a mock motor
    hwbridge::HBridgeDRV8801 motor;
    // Create a stepper control loop
    StepperControlLoop stepper_control_loop(motor, motor, clock);
    // Assert that the angle is the same as the previous angle
    std::pair<float, float> current_scalars = stepper_control_loop.determineCurrentSetpointScalars(315.0f);
    EXPECT_NEAR(current_scalars.first, 1.0f / std::sqrt(2.0f), 0.01);
    EXPECT_NEAR(current_scalars.second, -1.0f / std::sqrt(2.0f), 0.01);
}

}  // namespace control_loop