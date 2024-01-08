#include "bridge_3phase.hpp"
#include "brushless_foc_control_loop.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math.h"
#include "math_foc.hpp"
#include "mock_bridge_3phase.hpp"
#include "mock_hal_clock.hpp"
#include "mock_rotor_estimator.hpp"
#include "util.hpp"

namespace control_loop {
using namespace ::testing;

NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;

class BrushlessFOCControlLoopTest : public BrushlessFOCControlLoop {
   public:
    hwbridge::MOCK_BRIDGE_3PHASE bridge;
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors;

    BrushlessFOCControlLoop::BrushlessFocControLoopParams foc_params_{
        .current_control_bandwidth_rad_per_sec = 0.0f,

        .phase_resistance = 1.0f,
        .phase_resistance_valid = true,

        .phase_inductance = 1.0f,
        .phase_inductance_valid = true,

        .pm_flux_linkage = 0.0f,
        .disable_ki = false,

        .speed_to_iq_gain = 0.0f,
        .i_d_reference_default = 0.0f,
        .pwm_control_type = BldcFoc::BrushlessFocPwmControlType::SPACE_VECTOR,
    };

    BrushlessFOCControlLoop::Params test_params_{
        .foc_params = foc_params_,
        .open_loop_theta_velocity = 1.0f,
        .open_loop_quadrature_voltage = 1.0f,
        .phase_inductance_estimator_params = {.brake_duration = 1, .measurement_duration = 100},
        .phase_resistance_estimator_params = {.brake_duration = 1,
                                              .measurement_duration = 10000,
                                              .target_current = 1,
                                              .current_tolerance = 0.1,
                                              .max_voltage = 0.0f,
                                              .current_kp = 1.0f,
                                              .current_ki = 0.0f},
    };

    BrushlessFOCControlLoopTest(bldc_rotor_estimator::ElectricalRotorPosEstimator& rotor_position_estimator,
                                basilisk_hal::HAL_CLOCK& clock)
        : BrushlessFOCControlLoop(bridge, clock, &rotor_sensors, 1), rotor_sensors(&rotor_position_estimator) {}

    // Make the private functions public so we can test them
    using BrushlessFOCControlLoop::get_desired_state;
    using BrushlessFOCControlLoop::update_foc_inputs;
    using BrushlessFOCControlLoop::update_rotor_position_estimator;
};

// Test the state machine transition from stop to start, and a time out makes it go back to stop
TEST(BrushlessFOCControlLoopTest, test_stop_to_start_to_run) {
    // Create a mock rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR> sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::ElectricalRotorPosEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    EXPECT_EQ(
        test_control_loop.get_desired_state(0, BrushlessFOCControlLoop::State::STOP, test_control_loop.test_params_, status),
        BrushlessFOCControlLoop::State::STOP);

    // Call the desired state function with a time of foc_start_timeout_period -1, time at start of 0, and a motor speed of 0.1,
    // with the rotor estimator valid Ensure that the desired state is run
    EXPECT_EQ(
        test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::STOP, test_control_loop.test_params_, status),
        BrushlessFOCControlLoop::State::RUN);
}

// Test the state machine transition from run to stop
TEST(BrushlessFOCControlLoopTest, test_run_to_stop) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::ElectricalRotorPosEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessFOCControlLoop::State::RUN, test_control_loop.test_params_, status),
              BrushlessFOCControlLoop::State::STOP);
}

// Test that a phase current read failure sets the error appropriately
TEST(BrushlessFOCControlLoopTest, test_phase_current_read_failure) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    // Expect the phase current to be read
    EXPECT_CALL(test_control_loop.bridge, read_phase_current(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Run the FOC update input function
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;
    const utime_t time = 1U;
    math::alpha_beta_t V_alpha_beta;
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[1] = {&rotor_sensor};
    math::direct_quad_t i_direct_quad_ref;
    FOCController::FOCInputs foc_inputs =
        test_control_loop.update_foc_inputs(time, 0, rotor_sensors, 1, test_control_loop.bridge, status, V_alpha_beta,
                                            &test_control_loop.test_params_, i_direct_quad_ref);

    // Expect that the error is set
    EXPECT_TRUE(status.has_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::PHASE_CURRENT_READ_FAILURE));
    EXPECT_FALSE(foc_inputs.current_measurements_valid);
}

// Test the FOC update input function nominal case - no error
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_nominal) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    // Expect the phase current to be read
    const hwbridge::Bridge3Phase::phase_current_t phase_current = {1.0f, 1.0f, 1.0f};
    EXPECT_CALL(test_control_loop.bridge, read_phase_current(_))
        .WillOnce(DoAll(SetArgReferee<0>(phase_current), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the bus voltage to be read
    const float bus_voltage = 1.0f;
    EXPECT_CALL(test_control_loop.bridge, read_bus_voltage(_))
        .WillOnce(DoAll(SetArgReferee<0>(bus_voltage), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the rotor position to be read
    EXPECT_CALL(rotor_sensor, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(M_PI / 2.0f), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the rotor status to be valid
    EXPECT_CALL(rotor_sensor, is_estimation_valid()).WillOnce(Return(true));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;
    V_alpha_beta.alpha = 1.0f;
    V_alpha_beta.beta = 1.0f;

    // run the foc update input function
    const utime_t time = 1U;
    // Define a rotor sensor array
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[1] = {&rotor_sensor};
    math::direct_quad_t i_direct_quad_ref;
    i_direct_quad_ref.direct = 1.0f;
    i_direct_quad_ref.quadrature = 1.0f;
    FOCController::FOCInputs foc_inputs =
        test_control_loop.update_foc_inputs(time, 0, rotor_sensors, 1, test_control_loop.bridge, status, V_alpha_beta,
                                            &test_control_loop.test_params_, i_direct_quad_ref);

    // Sanity check that the variables are passed through

    // Expect the timestamp to be the time given
    EXPECT_EQ(foc_inputs.timestamp, time);

    // Expect the electrical angle to be M_PI/2
    EXPECT_FLOAT_EQ(foc_inputs.theta_e, M_PI / 2.0f);

    // Expect the idq currents to be what a clarke and park transform would give
    math::alpha_beta_t I_alpha_beta = math::clarke_transform(phase_current.u, phase_current.v, phase_current.w);
    math::direct_quad_t I_dq = math::park_transform(I_alpha_beta.alpha, I_alpha_beta.beta, foc_inputs.theta_e);
    EXPECT_FLOAT_EQ(foc_inputs.i_alpha_beta.alpha, I_alpha_beta.alpha);
    EXPECT_FLOAT_EQ(foc_inputs.i_alpha_beta.beta, I_alpha_beta.beta);
    EXPECT_FLOAT_EQ(foc_inputs.i_direct_quad.direct, I_dq.direct);
    EXPECT_FLOAT_EQ(foc_inputs.i_direct_quad.quadrature, I_dq.quadrature);
    EXPECT_TRUE(foc_inputs.current_measurements_valid);

    // Expect the bus voltage to be passed through
    EXPECT_FLOAT_EQ(foc_inputs.bus_voltage, bus_voltage);
    EXPECT_TRUE(foc_inputs.bus_voltage_valid);

    // Expect the idq reference currents to be passed through
    EXPECT_EQ(foc_inputs.i_direct_quad_ref, i_direct_quad_ref);

    // Expect the rotor status to be valid
    EXPECT_TRUE(foc_inputs.rotor_position_valid);

    // Expect the dt to be passed through
    EXPECT_FLOAT_EQ(foc_inputs.dt, 1 / (1e6));

    // Test the PWM control type
    EXPECT_EQ(foc_inputs.pwm_control_type, test_control_loop.test_params_.foc_params.pwm_control_type);

    // Test the open loop theta velocity and voltage
    EXPECT_EQ(foc_inputs.open_loop_theta_velocity, test_control_loop.test_params_.open_loop_theta_velocity);
    EXPECT_EQ(foc_inputs.open_loop_quadrature_voltage, test_control_loop.test_params_.open_loop_quadrature_voltage);

    EXPECT_EQ(status, ControlLoop::ControlLoopBaseStatus::OK);
}

// Test that the FOC update input function sets the rotor status to invalid if the rotor sensor is invalid
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_invalid_rotor_sensor) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor, is_estimation_valid()).WillOnce(Return(false));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;

    // run the foc update input function
    const utime_t time = 1U;
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[1] = {&rotor_sensor};
    math::direct_quad_t i_direct_quad_ref;
    FOCController::FOCInputs foc_inputs =
        test_control_loop.update_foc_inputs(time, 0, rotor_sensors, 1, test_control_loop.bridge, status, V_alpha_beta,
                                            &test_control_loop.test_params_, i_direct_quad_ref);

    // expect that the rotor position is invalid
    EXPECT_FALSE(foc_inputs.rotor_position_valid);

    // expect that an error is set in the status
    EXPECT_TRUE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR));

    // Now, call the function again, but instead of estimation invalid, we make the update function fail
    // Expect the rotor status to be invalid with the warning set (invalid because no other rotor estimator is available)
    EXPECT_CALL(rotor_sensor, update(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // run the foc update input function
    foc_inputs = test_control_loop.update_foc_inputs(time, 0, rotor_sensors, 1, test_control_loop.bridge, status, V_alpha_beta,
                                                     &test_control_loop.test_params_, i_direct_quad_ref);

    // expect that the rotor position is invalid
    EXPECT_FALSE(foc_inputs.rotor_position_valid);

    // expect that the warning is set in the status
    EXPECT_TRUE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE));
}

// Test that if 2 rotor estimators are available, the one with the highest priority is used
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_multiple_rotor_sensors) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor1;
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor2;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor1, mock_clock);

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor1, is_estimation_valid()).WillOnce(Return(true));

    // This position should return M_PI/3

    EXPECT_CALL(rotor_sensor1, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(M_PI / 3.0f), Return(app_hal_status_E::APP_HAL_OK)));

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor2, is_estimation_valid()).WillOnce(Return(true));

    // This position should return M_PI/2

    EXPECT_CALL(rotor_sensor2, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(M_PI / 2.0f), Return(app_hal_status_E::APP_HAL_OK)));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;

    // Give random phase currents
    const hwbridge::Bridge3Phase::phase_current_t phase_current = {1.0f, 1.0f, 1.0f};

    // run the foc update input function
    const utime_t time = 1U;
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
    float angle = 0;

    // Create an array of rotor sensors
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[2] = {&rotor_sensor1, &rotor_sensor2};

    test_control_loop.update_rotor_position_estimator(estimator_inputs, time, phase_current, &test_control_loop.test_params_,
                                                      V_alpha_beta, status, angle, rotor_sensors, 2);

    // expect that no error is set in the status
    EXPECT_FALSE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR));

    // expect the angle to be M_PI/3
    EXPECT_FLOAT_EQ(angle, M_PI / 3.0f);
}

// Test that if 2 rotor estimators are available and the first one fails, the second one is used
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_multiple_rotor_sensors_fail_first) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor1;
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor2;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor1, mock_clock);

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor1, is_estimation_valid()).WillOnce(Return(false));

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor2, is_estimation_valid()).WillOnce(Return(true));

    // This position should return M_PI/2
    EXPECT_CALL(rotor_sensor2, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(M_PI / 2.0f), Return(app_hal_status_E::APP_HAL_OK)));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;

    // Give random phase currents
    const hwbridge::Bridge3Phase::phase_current_t phase_current = {1.0f, 1.0f, 1.0f};

    // run the foc update input function
    const utime_t time = 1U;
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
    float angle = 0;

    // Create an array of rotor sensors
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[2] = {&rotor_sensor1, &rotor_sensor2};

    test_control_loop.update_rotor_position_estimator(estimator_inputs, time, phase_current, &test_control_loop.test_params_,
                                                      V_alpha_beta, status, angle, rotor_sensors, 2);

    // expect that no error is set in the status
    EXPECT_FALSE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR));

    // expect the angle to be M_PI/2
    EXPECT_FLOAT_EQ(angle, M_PI / 2.0f);
}

// If an update fails on the first rotor sensor, but the second one is valid, the second one should be used. Ensure the warning is
// set
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_multiple_rotor_sensors_fail_first_update) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor1;
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor2;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor1, mock_clock);

    // Expect the rotor status to be invalid with the warning set (invalid because no other rotor estimator is available)
    EXPECT_CALL(rotor_sensor1, update(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    // Expect the rotor status to be invalid
    EXPECT_CALL(rotor_sensor2, is_estimation_valid()).WillOnce(Return(true));

    // This position should return M_PI/2
    EXPECT_CALL(rotor_sensor2, get_rotor_position(_))
        .WillOnce(DoAll(SetArgReferee<0>(M_PI / 2.0f), Return(app_hal_status_E::APP_HAL_OK)));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;

    // Give random phase currents
    const hwbridge::Bridge3Phase::phase_current_t phase_current = {1.0f, 1.0f, 1.0f};

    // run the foc update input function
    const utime_t time = 1U;
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
    float angle = 0;

    // Create an array of rotor sensors
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[2] = {&rotor_sensor1, &rotor_sensor2};

    test_control_loop.update_rotor_position_estimator(estimator_inputs, time, phase_current, &test_control_loop.test_params_,
                                                      V_alpha_beta, status, angle, rotor_sensors, 2);

    // expect that no error is set in the status
    EXPECT_FALSE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR));

    // expect the angle to be M_PI/2
    EXPECT_FLOAT_EQ(angle, M_PI / 2.0f);

    // expect the warning to be set
    EXPECT_TRUE(status.has_warning(BrushlessFOCControlLoop::BrushlessFOCControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE));
}

// Test rotor position estimator update pass through (ensure that the variables are passed to the estimator inputs as a sanity
// check)
TEST(BrushlessFOCControlLoopTest, test_foc_update_input_rotor_position_estimator_update_pass_through) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    // Expect the rotor status to be invalid
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs_received;
    EXPECT_CALL(rotor_sensor, update(_))
        .WillOnce(DoAll(SaveArg<0>(&estimator_inputs_received), Return(app_hal_status_E::APP_HAL_OK)));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;

    // Give random phase currents
    const hwbridge::Bridge3Phase::phase_current_t phase_current = {1.0f, 1.0f, 1.0f};

    // run the foc update input function
    const utime_t time = 1U;
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
    float angle = 0;

    // Create an array of rotor sensors
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[1] = {&rotor_sensor};

    test_control_loop.update_rotor_position_estimator(estimator_inputs, time, phase_current, &test_control_loop.test_params_,
                                                      V_alpha_beta, status, angle, rotor_sensors, 1);

    // Expect the current time to be passed through
    EXPECT_EQ(estimator_inputs_received.time, time);
    // Expect the phase currents to be passed through
    EXPECT_EQ(estimator_inputs_received.phase_current, phase_current);
    // Expect the V_alpha_beta to be passed through
    EXPECT_EQ(estimator_inputs_received.V_alpha_beta, V_alpha_beta);
    // Expect the PM flux linkage to be passed through
    EXPECT_EQ(estimator_inputs_received.pm_flux_linkage, test_control_loop.test_params_.foc_params.pm_flux_linkage);
    // Expect the phase resistance to be passed through
    EXPECT_EQ(estimator_inputs_received.phase_resistance, test_control_loop.test_params_.foc_params.phase_resistance);
    // Expect the phase inductance to be passed through
    EXPECT_EQ(estimator_inputs_received.phase_inductance, test_control_loop.test_params_.foc_params.phase_inductance);
}

// Test that a bad bus voltage read sets the error appropriately
TEST(BrushlessFOCControlLoopTest, test_bus_voltage_read_failure) {
    // Create a mock absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    // Expect the bus voltage to be read
    EXPECT_CALL(test_control_loop.bridge, read_bus_voltage(_)).WillOnce(Return(app_hal_status_E::APP_HAL_ERROR));

    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status;

    // Give a random Valpha and Vbeta
    math::alpha_beta_t V_alpha_beta;
    V_alpha_beta.alpha = 1.0f;
    V_alpha_beta.beta = 1.0f;

    // run the foc update input function
    const utime_t time = 1U;
    // Define a rotor sensor array
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_sensors[1] = {&rotor_sensor};
    math::direct_quad_t i_direct_quad_ref;
    FOCController::FOCInputs foc_inputs =
        test_control_loop.update_foc_inputs(time, 0, rotor_sensors, 1, test_control_loop.bridge, status, V_alpha_beta,
                                            &test_control_loop.test_params_, i_direct_quad_ref);

    EXPECT_FALSE(foc_inputs.rotor_position_valid);

    // Expect that the error is set
    EXPECT_TRUE(status.has_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::BUS_VOLTAGE_READ_FAILURE));
}

// Test that the desired state is calibrating if either the phase resistance or phase inductance is indicated as invalid
TEST(BrushlessFOCControlLoopTest, test_desired_state_calibrating) {
    // Create an absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    BrushlessFOCControlLoop::Params test_params = test_control_loop.test_params_;
    test_params.foc_params.phase_inductance_valid = false;

    // Get the desired state
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    BrushlessFOCControlLoop::State desired_state =
        test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::STOP, test_params, status);

    // Expect the desired state to be calibrating
    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::CALIBRATION);

    // Set the phase inductance to be valid, and the phase resistance to be invalid
    test_params.foc_params.phase_inductance_valid = true;
    test_params.foc_params.phase_resistance_valid = false;

    // Get the desired state
    desired_state = test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::STOP, test_params, status);

    // Expect the desired state to be calibrating
    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::CALIBRATION);
}

// Test calibration -> run transition if both the phase resistance and phase inductance are valid
TEST(BrushlessFOCControlLoopTest, test_desired_state_calibration_to_run) {
    // Create an absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    BrushlessFOCControlLoop::Params test_params = test_control_loop.test_params_;

    // Get the desired state
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    BrushlessFOCControlLoop::State desired_state =
        test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::CALIBRATION, test_params, status);

    // Expect the desired state to be run
    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::RUN);
}

// Test calibration to stop transition if both are valid but the iq reference is 0
TEST(BrushlessFOCControlLoopTest, test_desired_state_calibration_to_stop_after_valid_calibration) {
    // Create an absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    BrushlessFOCControlLoop::Params test_params = test_control_loop.test_params_;

    // Set the iq reference to 0
    test_params.open_loop_quadrature_voltage = 0.0f;

    // Get the desired state
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    BrushlessFOCControlLoop::State desired_state =
        test_control_loop.get_desired_state(0.0, BrushlessFOCControlLoop::State::CALIBRATION, test_params, status);

    // Expect the desired state to be run
    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::STOP);
}

// Test that a failed phase estimation causes the desired state to go from calibration to stop
TEST(BrushlessFOCControlLoopTest, test_desired_state_calibration_to_stop_after_failed_calibration) {
    // Create an absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    BrushlessFOCControlLoop::Params test_params = test_control_loop.test_params_;

    test_params.foc_params.phase_resistance_valid = true;
    test_params.foc_params.phase_inductance_valid = false;

    // Get the desired state
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    status.set_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::PHASE_INDUCTANCE_ESTIMATOR_FAILURE, true);
    BrushlessFOCControlLoop::State desired_state =
        test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::CALIBRATION, test_params, status);

    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::STOP);

    status.set_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::PHASE_INDUCTANCE_ESTIMATOR_FAILURE, false);
    status.set_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::PHASE_RESISTANCE_ESTIMATOR_FAILURE, true);

    // Get the desired state
    desired_state = test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::CALIBRATION, test_params, status);

    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::STOP);
}

// Test that if the current state is stop and a reference is given, but the status has an error, the desired state is stop
TEST(BrushlessFOCControlLoopTest, test_desired_state_stop_to_stop_after_error) {
    // Create an absolute rotor position estimator
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_ABSOLUTE_SENSOR> rotor_sensor;

    // instantiate a brushless foc control loop test class
    BrushlessFOCControlLoopTest test_control_loop(rotor_sensor, mock_clock);

    BrushlessFOCControlLoop::Params test_params = test_control_loop.test_params_;

    // Get the desired state
    BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus status = BrushlessFOCControlLoop::BrushlessFOCControlLoopStatus();
    status.set_error(BrushlessFOCControlLoop::BrushlessFOCControlLoopError::PHASE_INDUCTANCE_ESTIMATOR_FAILURE, true);
    BrushlessFOCControlLoop::State desired_state =
        test_control_loop.get_desired_state(0.1, BrushlessFOCControlLoop::State::STOP, test_params, status);

    EXPECT_EQ(desired_state, BrushlessFOCControlLoop::State::STOP);
}

}  // namespace control_loop