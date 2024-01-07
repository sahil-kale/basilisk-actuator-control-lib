#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"
#include "phase_resistance_estimator.hpp"

using namespace ::testing;

namespace hwbridge {
// Make a public version of the PhaseResistanceEstimatorController class so we can test the protected methods
class PhaseResistanceEstimatorControllerPublic : public PhaseResistanceEstimatorController {
   public:
    Params default_params = {0, 1};
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PhaseResistanceEstimatorControllerPublic(basilisk_hal::MOCK_HAL_CLOCK& clock, Params params)
        : PhaseResistanceEstimatorController(clock, params) {}
    PhaseResistanceEstimatorControllerPublic() : PhaseResistanceEstimatorController(mock_clock, default_params) {}
};

TEST(PhaseResistanceEstimatorControllerTest, test_not_started_to_error_if_measure_duration_is_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorController::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 0;

    // Run the controller
    PhaseResistanceEstimatorController controller(mock_clock, params);
    PhaseResistanceEstimatorController::Input input;
    PhaseResistanceEstimatorController::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is ERROR
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::ERROR);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);
    // Expect that the phase commands are all off
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, false};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Test that the state transitions from NOT_STARTED to BRAKE_ROTOR when the brake duration is non-zero
TEST(PhaseResistanceEstimatorControllerTest, test_not_started_to_brake_rotor_if_brake_duration_is_non_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorController::Params params;
    params.brake_duration = 1;
    params.measurement_duration = 1;

    // Run the controller
    PhaseResistanceEstimatorController controller(mock_clock, params);
    PhaseResistanceEstimatorController::Input input;
    PhaseResistanceEstimatorController::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);
    // Expect that the phase commands are set to brake
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Test that if the state is NOT_STARTED and the brake duration is zero, we transition straight to MEASUREMENT_IN_PROGRESS
TEST(PhaseResistanceEstimatorControllerTest, test_not_started_to_measurement_in_progress_if_brake_duration_is_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorController::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 1;

    // Run the controller
    PhaseResistanceEstimatorController controller(mock_clock, params);
    PhaseResistanceEstimatorController::Input input;
    PhaseResistanceEstimatorController::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);
}

// Expect that the brake rotor state transitions to MEASUREMENT_IN_PROGRESS after the brake duration has elapsed
TEST(PhaseResistanceEstimatorControllerTest, test_brake_rotor_to_measurement_in_progress_after_brake_duration) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorControllerPublic::Params params;
    params.brake_duration = 3;
    params.measurement_duration = 1;

    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    // Run the controller
    PhaseResistanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseResistanceEstimatorControllerPublic::Input input;
    PhaseResistanceEstimatorControllerPublic::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);
    // Expect that the phase commands are set to brake
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);

    // Set the clock to be just before the brake duration has elapsed
    const utime_t faked_time_2 = faked_time + params.brake_duration - 1;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_2));

    // Run the controller again
    result = controller.run_phase_resistance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);

    // Set the clock to be just after the brake duration has elapsed
    const utime_t faked_time_3 = faked_time + params.brake_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_3));

    // Run the controller again
    result = controller.run_phase_resistance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);
}

// Expect that the measurement in progress state transitions to ESTIMATE_COMPLETE after the measurement duration has elapsed
TEST(PhaseResistanceEstimatorControllerTest, test_measurement_in_progress_to_estimate_complete_after_measurement_duration) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 2;
    params.current_ki = 0.0f;
    params.current_kp = 1.0f;
    params.target_current = 1.0f;

    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    // Run the controller
    PhaseResistanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseResistanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 10.0f;
    input.phase_currents.u = 0.0f;
    PhaseResistanceEstimatorControllerPublic::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);

    // We should expect that the voltage set by the current controller is simply proportional gain * error
    // error is just target current as we have kept the input current at 0
    const float expected_voltage = params.current_kp * params.target_current;
    const float expected_duty_cycle = expected_voltage / (2 * input.bus_voltage) + 0.5f;
    const Bridge3Phase::phase_command_t expected_phase_command_u = {0.0f, true};  // Grounded
    const Bridge3Phase::phase_command_t expected_phase_command_v_w = {expected_duty_cycle, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command_u);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command_v_w);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command_v_w);

    // Set the clock to be just after the measurement duration has elapsed
    const utime_t faked_time_2 = faked_time + params.measurement_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_2));

    // Make the current the same as the target current
    input.phase_currents.u = params.target_current;
    result = controller.run_phase_resistance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::ESTIMATE_COMPLETE);
    // Check that the phase resistance is not valid
    EXPECT_TRUE(result.is_phase_resistance_valid);
    // Check that the phase resistance is correct
    const float expected_phase_resistance = expected_voltage / params.target_current * 2.0f / 3.0f;
    EXPECT_FLOAT_EQ(result.phase_resistance, expected_phase_resistance);

    // Check that the phase commands are all off
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, false};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Test that if we don't get to the target current within the measurement duration, we transition to ERROR
TEST(PhaseResistanceEstimatorController, test_current_not_within_margin) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseResistanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 2;
    params.current_ki = 0.0f;
    params.current_kp = 1.0f;
    params.target_current = 1.0f;
    params.measurement_duration = 2;

    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    // Run the controller
    PhaseResistanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseResistanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 10.0f;
    input.phase_currents.u = 0.0f;
    PhaseResistanceEstimatorControllerPublic::Result result = controller.run_phase_resistance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);

    // Set the clock to be just after the measurement duration has elapsed
    const utime_t faked_time_2 = faked_time + params.measurement_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_2));

    // Make the current the same as the target current minus the margin plus a little bit
    input.phase_currents.u = params.target_current - (params.current_tolerance + 0.01f);

    result = controller.run_phase_resistance_estimator(input);

    // Check that the state is ERROR
    EXPECT_EQ(result.state, PhaseResistanceEstimatorController::State::ERROR);
    // Check that the phase resistance is not valid
    EXPECT_FALSE(result.is_phase_resistance_valid);

    // Check that the phase commands are all off
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, false};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

}  // namespace hwbridge