#include "rotor_estimator.hpp"

#include "bridge_3phase.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math_util.hpp"
#include "mock_bridge_3phase.hpp"
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
    EXPECT_CALL(sector_sensor, get_electrical_angle(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1 * math::M_PI_FLOAT / 3.0), Return(APP_HAL_OK)));

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
    EXPECT_CALL(sector_sensor, get_electrical_angle(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(5 * math::M_PI_FLOAT / 3.0), Return(APP_HAL_OK)));

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
    EXPECT_CALL(sector_sensor, get_electrical_angle(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1 * math::M_PI_FLOAT / 3.0), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(500);

    float rotor_position = 0.0f;
    rotor_estimator.get_rotor_position(rotor_position);

    EXPECT_FLOAT_EQ(rotor_position, 1.0f * 2.0f * M_PI / 6.0f);

    EXPECT_CALL(sector_sensor, get_electrical_angle(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(1 * math::M_PI_FLOAT / 3.0), Return(APP_HAL_OK)));

    // Update the rotor position
    rotor_estimator.update(1000);

    rotor_estimator.get_rotor_position(rotor_position);

    EXPECT_FLOAT_EQ(rotor_position, 1.0f * 2.0f * M_PI / 6.0f);
}

class SensorlessRotorSectorSensor : public BldcSensorlessRotorSectorSensor {
   public:
    SensorlessRotorSectorSensor(hwbridge::Bridge3Phase& bridge, basilisk_hal::HAL_CLOCK& mock_clock)
        : BldcSensorlessRotorSectorSensor(bridge, mock_clock) {}

    float estimated_electrical_angle_ = 0.0f;

    // Make the protected functions public for testing
    using BldcSensorlessRotorSectorSensor::get_electrical_angle;
    using BldcSensorlessRotorSectorSensor::reset;
    using BldcSensorlessRotorSectorSensor::zero_crossing_detected;
};

// Test the sensorless sector estimator zero crossing detected function
TEST(RotorEstimatorTest, test_zero_crossing_detection) {
    // Make a mock bridge
    hwbridge::MOCK_BRIDGE_3PHASE mock_bridge;
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    SensorlessRotorSectorSensor sensorless_sensor(mock_bridge, mock_clock);

    // Test the zero crossing detection for commutation step 4
    // We expect HIGH, LOW, Z_FALLING

    // Make a fake bemf voltage struct
    hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage = {0.0f, 0.0f, 0.0f};

    // Load the bemf with a voltage of 1.0f, 0.0f, and 1.0f
    bemf_voltage.u = 1.0f;
    bemf_voltage.v = 0.0f;
    bemf_voltage.w = 1.0f;

    // With this, we should not detect a zero crossing
    EXPECT_FALSE(
        sensorless_sensor.zero_crossing_detected(bemf_voltage, control_loop::Bldc6StepCommutationTypes::commutation_steps[4]));

    // Load the bemf with a voltage of 1.0f, 0.0f, and 0.51f
    bemf_voltage.w = 0.51f;

    // With this, we should not detect a zero crossing
    EXPECT_FALSE(
        sensorless_sensor.zero_crossing_detected(bemf_voltage, control_loop::Bldc6StepCommutationTypes::commutation_steps[4]));

    // Load the bemf with a voltage of 1.0f, 0.0f, and 0.49f

    bemf_voltage.w = 0.49f;

    // With this, we should detect a zero crossing
    EXPECT_TRUE(
        sensorless_sensor.zero_crossing_detected(bemf_voltage, control_loop::Bldc6StepCommutationTypes::commutation_steps[4]));
}

// Test the sensorless sector estimator reporting a sector change for the same duration of time as when a zero crossing was
// detected
TEST(RotorEstimatorTest, sensorless_estimator_timing_switch) {
    // Make a mock bridge
    hwbridge::MOCK_BRIDGE_3PHASE mock_bridge;
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    SensorlessRotorSectorSensor sensorless_sensor(mock_bridge, mock_clock);

    // Expect a call for the clock time, set to 1
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1));
    // Reset the sensorless sensor
    sensorless_sensor.reset();

    // We should be in commutation step 0
    // Make a fake bemf voltage struct
    hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage = {0.0f, 0.0f, 0.0f};

    // In sector 0, we expect Z_FALLING, HIGH, LOW
    bemf_voltage.u = 1.0f;
    bemf_voltage.v = 1.0f;
    bemf_voltage.w = 0.0f;

    // Expect a call to the bridge to get the bemf voltage
    EXPECT_CALL(mock_bridge, read_bemf(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(bemf_voltage), Return(APP_HAL_OK)));

    // Expect a call for the clock time, set to 1
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1));

    float electrical_angle = 0.0f;
    sensorless_sensor.get_electrical_angle(electrical_angle);
    EXPECT_FLOAT_EQ(electrical_angle, 0.0f);

    // Now, set the bemf voltage to 0.49, 0.0, 1.0
    bemf_voltage.u = 0.49f;
    bemf_voltage.v = 0.0f;
    bemf_voltage.w = 1.0f;

    // Expect a call to the bridge to get the bemf voltage
    EXPECT_CALL(mock_bridge, read_bemf(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(bemf_voltage), Return(APP_HAL_OK)));

    // Expect a call for the clock time, set to 500
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(501));

    // Expect the sector to be 0
    sensorless_sensor.get_electrical_angle(electrical_angle);
    EXPECT_FLOAT_EQ(electrical_angle, 0.0f);

    // Expect a call to the bridge to get the bemf voltage
    EXPECT_CALL(mock_bridge, read_bemf(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(bemf_voltage), Return(APP_HAL_OK)));

    // Expect a call for the clock time, set to 999
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1000));

    // Expect the sector to be 0
    sensorless_sensor.get_electrical_angle(electrical_angle);
    EXPECT_FLOAT_EQ(electrical_angle, 0.0f);

    // Expect a call to the bridge to get the bemf voltage
    EXPECT_CALL(mock_bridge, read_bemf(_))  // _ allowing any param
        .WillOnce(DoAll(SetArgReferee<0>(bemf_voltage), Return(APP_HAL_OK)));

    // Expect a call for the clock time, set to 1000
    EXPECT_CALL(mock_clock, get_time_us()).WillOnce(Return(1001));

    // Expect the sector to be 1
    sensorless_sensor.get_electrical_angle(electrical_angle);
    EXPECT_FLOAT_EQ(electrical_angle, 1.0 * math::M_PI_FLOAT / 3.0);
}

}  // namespace bldc_rotor_estimator