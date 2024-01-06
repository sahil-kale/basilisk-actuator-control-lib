#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"
#include "phase_inductance_estimator.hpp"

namespace hwbridge {

using namespace ::testing;

// Create a class that extends the PhaseInductanceEstimatorController class and exposes the protected methods
class PhaseInductanceEstimatorControllerPublic : public PhaseInductanceEstimatorController {
   public:
    Params default_params = {0, 1};
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PhaseInductanceEstimatorControllerPublic(basilisk_hal::MOCK_HAL_CLOCK& clock, Params params)
        : PhaseInductanceEstimatorController(clock, params) {}
    PhaseInductanceEstimatorControllerPublic() : PhaseInductanceEstimatorController(mock_clock, default_params) {}
};

TEST(PhaseInductanceEstimatorControllerTest, test_not_started_to_error_if_measure_duration_is_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 0;

    // Run the controller
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is ERROR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::ERROR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);
    // Expect that the phase commands are all off
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, false};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Test that the state transitions from NOT_STARTED to BRAKE_ROTOR when the brake duration is non-zero
TEST(PhaseInductanceEstimatorControllerTest, test_not_started_to_brake_rotor_if_brake_duration_is_non_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 1;
    params.measurement_duration = 1;

    // Run the controller
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);
    // Expect that the phase commands are set to brake
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Expect that the brake rotor state transitions to MEASUREMENT_IN_PROGRESS after the brake duration has elapsed
TEST(PhaseInductanceEstimatorControllerTest, test_brake_rotor_to_measurement_in_progress_after_brake_duration) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 3;
    params.measurement_duration = 1;

    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    // Run the controller
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);
    // Expect that the phase commands are set to brake
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);

    // Set the clock to be just before the brake duration has elapsed
    const utime_t faked_time_2 = faked_time + params.brake_duration - 1;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_2));

    // Run the controller again
    result = controller.run_phase_inductance_estimator(input);

    // Check that the state is BRAKE_ROTOR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::BRAKE_ROTOR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);

    // Set the clock to be just after the brake duration has elapsed
    const utime_t faked_time_3 = faked_time + params.brake_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_3));

    // Run the controller again
    result = controller.run_phase_inductance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);

    // Expect that the phase commands are set to give a square wave (A = 0.0, B = 1.0, C = 1.0)
    const Bridge3Phase::phase_command_t expected_phase_command_a = {0.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_b = {1.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_c = {1.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command_a);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command_b);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command_c);
}

// Test that if the state is NOT_STARTED and the brake duration is zero, we transition straight to MEASUREMENT_IN_PROGRESS
TEST(PhaseInductanceEstimatorControllerTest, test_not_started_to_measurement_in_progress_if_brake_duration_is_zero) {
    NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 1;

    // Run the controller
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);

    // Expect that the phase commands are set to give a square wave (A = 0.0, B = 1.0, C = 1.0)
    const Bridge3Phase::phase_command_t expected_phase_command_a = {0.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_b = {1.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_c = {1.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command_a);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command_b);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command_c);
}

// Helper function to put the controller on the next run iteration to finish MEASUREMENT_IN_PROGRESS
void init_and_ready_to_finish_measurement_in_progress(basilisk_hal::MOCK_HAL_CLOCK& mock_clock,
                                                      PhaseInductanceEstimatorControllerPublic& controller,
                                                      PhaseInductanceEstimatorController::Params& params) {
    params.brake_duration = 0;
    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    PhaseInductanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 24.0f;
    input.phase_currents.u = 1.0f;
    input.phase_currents.v = 0.0f;
    input.phase_currents.w = 0.0f;

    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);

    // Set the clock to be just after the measurement duration has elapsed
    const utime_t faked_time_3 = faked_time + params.measurement_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_3));
    input.phase_currents.u = 1.0f;
}

// Test that we transition from MEASUREMENT_IN_PROGRESS to ESTIMATE_COMPLETE after the measurement duration has elapsed
// Further, verify that the phase inductance is valid and that the phase commands are all off
TEST(PhaseInductanceEstimatorControllerTest, test_measurement_in_progress_to_estimate_complete_after_measurement_duration) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    // Create a param struct with a measure duration of 0
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 100;  // 10kHz square wave

    const utime_t faked_time = 12;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time));

    // Run the controller
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 24.0f;
    input.phase_currents.u = 0.0f;
    input.phase_currents.v = 0.0f;
    input.phase_currents.w = 0.0f;

    PhaseInductanceEstimatorControllerPublic::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);

    // Expect that the phase commands are set to give a square wave (A = 0.0, B = 1.0, C = 1.0)
    const Bridge3Phase::phase_command_t expected_phase_command_a = {0.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_b = {1.0f, true};
    const Bridge3Phase::phase_command_t expected_phase_command_c = {1.0f, true};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command_a);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command_b);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command_c);

    // Set the clock to be just before the measurement duration has elapsed
    const utime_t faked_time_2 = faked_time + params.measurement_duration - 1;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_2));

    // Run the controller again
    result = controller.run_phase_inductance_estimator(input);
    EXPECT_EQ(result.phase_commands[0], expected_phase_command_a);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command_b);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command_c);

    // Check that the state is MEASUREMENT_IN_PROGRESS
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::MEASUREMENT_IN_PROGRESS);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);

    // Set the clock to be just after the measurement duration has elapsed
    const utime_t faked_time_3 = faked_time + params.measurement_duration;
    EXPECT_CALL(mock_clock, get_time_us()).WillRepeatedly(Return(faked_time_3));

    // Run the controller again
    input.phase_currents.u = 1.6f;
    result = controller.run_phase_inductance_estimator(input);

    // Check that the state is ESTIMATE_COMPLETE
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::ESTIMATE_COMPLETE);
    // Check that the phase inductance is valid
    EXPECT_TRUE(result.is_phase_inductance_valid);

    // The parameters for this test are such that the phase inductance should be 1mH
    EXPECT_FLOAT_EQ(result.phase_inductance, 0.001f);

    // Expect that the phase commands are all off
    const Bridge3Phase::phase_command_t expected_phase_command = {0.0f, false};
    EXPECT_EQ(result.phase_commands[0], expected_phase_command);
    EXPECT_EQ(result.phase_commands[1], expected_phase_command);
    EXPECT_EQ(result.phase_commands[2], expected_phase_command);
}

// Test that we transition from MEASUREMENT_IN_PROGRESS to ERROR if the current is 0
TEST(PhaseInductanceEstimatorControllerTest, test_measurement_in_progress_to_error_if_current_is_zero) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 1;
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    init_and_ready_to_finish_measurement_in_progress(mock_clock, controller, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 24.0f;
    input.phase_currents.u = 0.0f;
    input.phase_currents.v = 0.0f;
    input.phase_currents.w = 0.0f;

    PhaseInductanceEstimatorController::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is ERROR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::ERROR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);
}

// Test that we transition from MEASUREMENT_IN_PROGRESS to ERROR if the bus voltage is 0
TEST(PhaseInductanceEstimatorControllerTest, test_measurement_in_progress_to_error_if_bus_voltage_is_zero) {
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    PhaseInductanceEstimatorControllerPublic::Params params;
    params.brake_duration = 0;
    params.measurement_duration = 1;
    PhaseInductanceEstimatorControllerPublic controller(mock_clock, params);
    init_and_ready_to_finish_measurement_in_progress(mock_clock, controller, params);
    PhaseInductanceEstimatorControllerPublic::Input input;
    input.bus_voltage = 0.0f;
    input.phase_currents.u = 1.0f;
    input.phase_currents.v = 0.0f;
    input.phase_currents.w = 0.0f;

    PhaseInductanceEstimatorController::Result result = controller.run_phase_inductance_estimator(input);

    // Check that the state is ERROR
    EXPECT_EQ(result.state, PhaseInductanceEstimatorController::State::ERROR);
    // Check that the phase inductance is not valid
    EXPECT_FALSE(result.is_phase_inductance_valid);
}

}  // namespace hwbridge