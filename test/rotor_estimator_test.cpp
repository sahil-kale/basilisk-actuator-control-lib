#include "rotor_estimator.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"
#include "mock_rotor_estimator.hpp"

using namespace ::testing;

namespace bldc_rotor_estimator {
basilisk_hal::MOCK_HAL_CLOCK mock_clock;

TEST(RotorEstimatorTest, test_angle_one_for_one) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // Make a param struct for the rotor estimator
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall::BldcElectricalRotorPositionEstimatorFromHallParams_t
        params{
            .num_hall_updates_to_start = 10,
            .max_estimate_angle_overrun = 2.0f / 3.0f * M_PI,
            .enable_interpolation = true,
        };

    rotor_estimator.init(&params);

    // Expect a call to get the sector position and ensure the reference is updated to return 3
    EXPECT_CALL(sector_sensor, get_sector(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(500);

    float rotor_position = 0.0f;
    rotor_estimator.get_rotor_position(rotor_position);

    EXPECT_FLOAT_EQ(rotor_position, 2.0f * 2.0f * M_PI / 6.0f);

    // Expect raw float angle to be 1.0f * 2.0f * M_PI / 6.0f
    float raw_hall_angle = 0.0f;
    rotor_estimator.get_raw_hall_angle(raw_hall_angle);
    EXPECT_FLOAT_EQ(raw_hall_angle, 1.0f * 2.0f * M_PI / 6.0f);

    // Expect the velocity to be (PI/3 rad) / (500 us) = 2094.395 rad/s
    // However, we use compensated velocity, which is 2x the velocity due to the sudden velocity jump
    // this is done to avoid discontinuities in the velocity
    const float actual_velocity = M_PI / (3.0f * (500.0f / mock_clock.kMicrosecondsPerSecond));
    const float compensated_velocity = 1.0f * actual_velocity;
    float rotor_velocity = 0.0f;
    rotor_estimator.get_rotor_velocity(rotor_velocity);
    EXPECT_FLOAT_EQ(rotor_velocity, compensated_velocity);
}

TEST(RotorEstimatorTest, test_angle_underflow) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall::BldcElectricalRotorPositionEstimatorFromHallParams_t
        params{
            .num_hall_updates_to_start = 10,
            .max_estimate_angle_overrun = 2.0f / 3.0f * M_PI,
            .enable_interpolation = true,
        };

    rotor_estimator.init(&params);

    // Expect a call to get the sector position and ensure the reference is updated to return 3
    EXPECT_CALL(sector_sensor, get_sector(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(5), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(500);

    float rotor_position = 0.0f;
    rotor_estimator.get_rotor_position(rotor_position);

    // Expect raw float angle to be 1.0f * 2.0f * M_PI / 6.0f
    float raw_hall_angle = 0.0f;
    rotor_estimator.get_raw_hall_angle(raw_hall_angle);
    EXPECT_FLOAT_EQ(raw_hall_angle, 5.0f * 2.0f * M_PI / 6.0f);

    // Expect the velocity to be -(PI/3 rad) / (500 us) = 2094.395 rad/s
    // However, we use compensated velocity, which is 2x the velocity due to the sudden velocity jump

    const float actual_velocity = -M_PI / (3.0f * (500.0f / mock_clock.kMicrosecondsPerSecond));
    const float compensated_velocity = 1.0f * actual_velocity;
    float rotor_velocity = 0.0f;
    rotor_estimator.get_rotor_velocity(rotor_velocity);
    EXPECT_FLOAT_EQ(rotor_velocity, compensated_velocity);
}

// Test that interpolation disabled reports the raw hall angle
// Create a mock rotor sensor
TEST(RotorEstimatorTest, test_angle_interpolation_disabled) {
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // Make a param struct for the rotor estimator
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimatorFromHall::BldcElectricalRotorPositionEstimatorFromHallParams_t
        params{
            .num_hall_updates_to_start = 10,
            .max_estimate_angle_overrun = 2.0f / 3.0f * M_PI,
            .enable_interpolation = false,
        };

    rotor_estimator.init(&params);

    // Expect a call to get the sector position and ensure the reference is updated to return 3
    EXPECT_CALL(sector_sensor, get_sector(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(500);

    float rotor_position = 0.0f;
    rotor_estimator.get_rotor_position(rotor_position);

    EXPECT_FLOAT_EQ(rotor_position, 1.0f * 2.0f * M_PI / 6.0f);

    EXPECT_CALL(sector_sensor, get_sector(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(1000);

    rotor_estimator.get_rotor_position(rotor_position);

    EXPECT_FLOAT_EQ(rotor_position, 1.0f * 2.0f * M_PI / 6.0f);
}

}  // namespace bldc_rotor_estimator