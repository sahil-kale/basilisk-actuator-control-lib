#include "pid.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace pid {
// Test that setting the gains works and the same gains are returned
TEST(PIDTest, SetGainsTest) {
    PID<float> pid(1, 2, 3, -10, 10, 0);
    EXPECT_EQ(pid.get_kp(), 1);
    EXPECT_EQ(pid.get_ki(), 2);
    EXPECT_EQ(pid.get_kd(), 3);
}

// Test that the control value proportional gain is calculated correctly
TEST(PIDTest, ProportionalGainTest) {
    PID<float> pid(1, 0, 0, -10, 10, 0);
    EXPECT_EQ(pid.get_kp(), 1.0);
    EXPECT_EQ(pid.calculate(0, 1.0), 1);
    EXPECT_EQ(pid.calculate(1, 0), -1);
    EXPECT_EQ(pid.calculate(1, 1), 0);
}

// Test the integral gain
TEST(PIDTest, IntegralGainTest) {
    PID<float> pid(0, 1, 0, -10, 10, 10);
    EXPECT_EQ(pid.get_ki(), 1.0);
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    EXPECT_EQ(pid.calculate(0, 1), 0);
    EXPECT_EQ(pid.calculate(0, 1), 1);
}

// Test the derivative gain
TEST(PIDTest, DerivativeGainTest) {
    PID<float> pid(0, 0, 1, -10, 10, 0);
    EXPECT_EQ(pid.get_kd(), 1.0);
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    EXPECT_EQ(pid.calculate(0, 1), 2);
    EXPECT_EQ(pid.calculate(0, 1), 0);
    EXPECT_EQ(pid.calculate(0, 5), 4);
}

// Test the integral windup
TEST(PIDTest, IntegralWindupTest) {
    PID<float> pid(0, 1, 0, -10, 50, 10);
    EXPECT_EQ(pid.get_ki(), 1.0);
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    EXPECT_EQ(pid.calculate(0, 1), 0);
    EXPECT_EQ(pid.calculate(0, 1), 1);
    EXPECT_EQ(pid.calculate(0, 1), 2);
    EXPECT_EQ(pid.calculate(0, 1), 3);
    EXPECT_EQ(pid.calculate(0, 1), 4);
    EXPECT_EQ(pid.calculate(0, 1), 5);
    EXPECT_EQ(pid.calculate(0, 1), 6);
    EXPECT_EQ(pid.calculate(0, 1), 7);
    EXPECT_EQ(pid.calculate(0, 1), 8);
    EXPECT_EQ(pid.calculate(0, 1), 9);
    EXPECT_EQ(pid.calculate(0, 1), 10);
    EXPECT_EQ(pid.calculate(0, 1), 10);
}

// Test the max output
TEST(PIDTest, MaxOutputTest) {
    PID<float> pid(1, 0, 0, -10, 10, 0);
    EXPECT_EQ(pid.calculate(0.0, 1000.0), 10);
}

// Test the min output
TEST(PIDTest, MinOutputTest) {
    PID<float> pid(1, 0, 0, -10, 10, 0);
    EXPECT_EQ(pid.calculate(0.0, -1000.0), -10);
}
}  // namespace pid