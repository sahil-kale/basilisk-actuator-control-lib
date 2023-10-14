#include "pid.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"

namespace pid {

using namespace ::testing;

// Test that setting the gains works and the same gains are returned
TEST(PIDTest, SetGainsTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(1, 2, 3, -10, 10, 0, mock_clock);
    EXPECT_EQ(pid.get_kp(), 1);
    EXPECT_EQ(pid.get_ki(), 2);
    EXPECT_EQ(pid.get_kd(), 3);
}

// Test that the control value proportional gain is calculated correctly
TEST(PIDTest, ProportionalGainTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(1, 0, 0, -10, 10, 0, mock_clock);
    EXPECT_EQ(pid.get_kp(), 1.0);
    // Expect a call to get_time_us() to return 1
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1));
    EXPECT_EQ(pid.calculate(0, 1.0), 1);

    // Expect a call to get_time_us() to return 2
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(2));
    EXPECT_EQ(pid.calculate(1, 0), -1);

    // Expect a call to get_time_us() to return 3
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(3));
    EXPECT_EQ(pid.calculate(1, 1), 0);
}

// Test the integral gain
TEST(PIDTest, IntegralGainTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(0, 1, 0, -10, 10, 10, mock_clock);
    EXPECT_EQ(pid.get_ki(), 1.0);
    // Expect a call to get_time_us() to return 1*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    // Expect a call to get_time_us() to return 2*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(2 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 0);
    // Expect a call to get_time_us() to return 3*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(3 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 1);
}

// Test the derivative gain
TEST(PIDTest, DerivativeGainTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(0, 0, 1, -10, 10, 0, mock_clock);
    EXPECT_EQ(pid.get_kd(), 1.0);
    // Expect a call to get_time_us() to return 1*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    // Expect a call to get_time_us() to return 2*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(2 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 2);
    // Expect a call to get_time_us() to return 3*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(3 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 0);
    // Expect a call to get_time_us() to return 4*basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(4 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 5), 4);
}

// Test the integral windup
TEST(PIDTest, IntegralWindupTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(0, 1, 0, -10, 50, 3, mock_clock);
    EXPECT_EQ(pid.get_ki(), 1.0);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(1.0, 0.0), -1);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(2 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 0);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(3 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 1);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(4 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 2);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(5 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 3);
    // Expect a call to get_time_us()
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(6 * basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond));
    EXPECT_EQ(pid.calculate(0, 1), 3);
}

// Test the max output
TEST(PIDTest, MaxOutputTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(1, 0, 0, -10, 10, 0, mock_clock);
    // Expect a call to get_time_us() to return 1
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1));
    EXPECT_EQ(pid.calculate(0.0, 1000.0), 10);
}

// Test the min output
TEST(PIDTest, MinOutputTest) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PID<float> pid(1, 0, 0, -10, 10, 0, mock_clock);
    // Expect a call to get_time_us() to return 1
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1));
    EXPECT_EQ(pid.calculate(0.0, -1000.0), -10);
}
}  // namespace pid