#include "brushless_6step_control_loop.hpp"

#include "6step_util.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_bridge_3phase.hpp"
#include "mock_rotor_estimator.hpp"

using namespace ::testing;
namespace control_loop {

TEST(Brushless6StepControlLoopTest, test_bad_phase_voltage_warning) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the phase voltage read to fail
    EXPECT_CALL(bridge, read_phase_voltage(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Run the control loop
    control_loop.run(1.0f);

    // Get the status
    auto status = control_loop.get_status();

    // Check that the warning is set
    EXPECT_TRUE(status.has_warning(Brushless6StepControlLoop::Brushless6StepWarning::PHASE_VOLTAGE_READ_FAILURE));
}

// Test that calling init resets the rotor position estimator
TEST(Brushless6StepControlLoopTest, test_init_resets_rotor_position_estimator) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the rotor position estimator to be reset
    EXPECT_CALL(rotor_sensor, reset_estimation());

    // Call init
    control_loop.init();
}

// Expect a rotor position estimator failure if the rotor position estimator fails. Further, expect the duty cycles to be set to 0
TEST(Brushless6StepControlLoopTest, test_rotor_position_estimator_failure) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the rotor position estimator to fail
    EXPECT_CALL(rotor_sensor, update(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Expect a call to set the duty cycles to 0
    hwbridge::Bridge3Phase::phase_command_t zero_duty_cycle;
    zero_duty_cycle.duty_cycle_high_side = 0.0f;
    zero_duty_cycle.enable = false;

    EXPECT_CALL(bridge, set_phase(zero_duty_cycle, zero_duty_cycle, zero_duty_cycle))
        .WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(1.0f);

    // Get the status
    auto status = control_loop.get_status();

    // Check that the error is set
    EXPECT_TRUE(status.has_error(Brushless6StepControlLoop::Brushless6StepError::ROTOR_POSITION_ESTIMATOR_UPDATE_FAILURE));
}

// Test a rotor estimator get electrical angle failure
TEST(Brushless6StepControlLoopTest, test_rotor_position_estimator_get_angle_failure) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the rotor position estimator to fail
    EXPECT_CALL(rotor_sensor, get_rotor_position(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Expect a call to set the duty cycles to 0
    hwbridge::Bridge3Phase::phase_command_t zero_duty_cycle;
    zero_duty_cycle.duty_cycle_high_side = 0.0f;
    zero_duty_cycle.enable = false;

    EXPECT_CALL(bridge, set_phase(zero_duty_cycle, zero_duty_cycle, zero_duty_cycle))
        .WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(1.0f);

    // Get the status
    auto status = control_loop.get_status();

    // Check that the error is set
    EXPECT_TRUE(status.has_error(Brushless6StepControlLoop::Brushless6StepError::ROTOR_POSITION_ESTIMATOR_GET_ANGLE_FAILURE));
}

// Ensure that when the estimator is updated, the phase voltages are passed to the estimator alongside the commutation step
TEST(Brushless6StepControlLoopTest, test_estimator_update_with_phase_voltage) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the phase voltage to be read
    const hwbridge::Bridge3Phase::phase_voltage_t phase_voltage = {1.0f, 1.0f, 1.0f};
    EXPECT_CALL(bridge, read_phase_voltage(_))
        .WillOnce(DoAll(SetArgReferee<0>(phase_voltage), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the rotor position estimator to be updated with the phase voltage
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs inputs;
    EXPECT_CALL(rotor_sensor, update(_)).WillOnce(DoAll(SaveArg<0>(&inputs), Return(app_hal_status_E::APP_HAL_OK)));

    // Run the control loop
    control_loop.run(1.0f);

    // Expect the phase voltage to be passed to the estimator
    EXPECT_EQ(inputs.phase_voltage, phase_voltage);
}

// Test electrical angle to commutation step duty cycle generation. Test step 0 with a speed of 0.5
// We should expect control loop OK, and the duty cycles to be set to 0.5, 0, 0
TEST(Brushless6StepControlLoopTest, test_6_step_duty_cycle_generation) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the rotor position estimator to return 0
    EXPECT_CALL(rotor_sensor, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(0.0f), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the duty cycles to be set to whatever the determine_inverter_duty_cycles_trap function returns
    hwbridge::Bridge3Phase::phase_command_t phase_command[3];
    Bldc6Step::determine_inverter_duty_cycles_trap(phase_command, Bldc6Step::commutation_steps[0], 0.5f);

    EXPECT_CALL(bridge, set_phase(phase_command[0], phase_command[1], phase_command[2]))
        .WillOnce(Return(app_hal_status_E::APP_HAL_OK));

    // Run the control loop
    control_loop.run(0.5f);

    // Get the status
    auto status = control_loop.get_status();

    // Check that the status is OK
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::OK);
}

// Test that if the duty cycle set fails, the error is set
TEST(Brushless6StepControlLoopTest, test_duty_cycle_set_failure) {
    // Create a mock bridge
    NiceMock<hwbridge::MOCK_BRIDGE_3PHASE> bridge;

    // Create a mock absolute rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // Create the control loop
    control_loop::Brushless6StepControlLoop control_loop(bridge, rotor_sensor);

    // Expect the rotor position estimator to return 0
    EXPECT_CALL(rotor_sensor, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(0.0f), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the duty cycles to be set to whatever the determine_inverter_duty_cycles_trap function returns
    hwbridge::Bridge3Phase::phase_command_t phase_command[3];
    Bldc6Step::determine_inverter_duty_cycles_trap(phase_command, Bldc6Step::commutation_steps[0], 0.5f);

    EXPECT_CALL(bridge, set_phase(phase_command[0], phase_command[1], phase_command[2]))
        .WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Run the control loop
    control_loop.run(0.5f);

    // Get the status
    auto status = control_loop.get_status();

    // Check that the status has the BRIDGE_DUTY_CYCLE_SET_FAILURE error
    EXPECT_TRUE(status.has_error(Brushless6StepControlLoop::Brushless6StepError::BRIDGE_DUTY_CYCLE_SET_FAILURE));
}

}  // namespace control_loop