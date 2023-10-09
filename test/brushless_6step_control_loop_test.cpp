#include "brushless_6step_control_loop.hpp"

#include "mock_bridge_3phase.hpp"
#include "mock_hal_adc.hpp"
#include "mock_hal_clock.hpp"
#include "param_service.hpp"

namespace control_loop {
using namespace ::testing;

class Brushless6StepControlLoopTest {
   public:
    // Define a mock HAL clock object
    basilisk_hal::MOCK_HAL_CLOCK mock_clock_;

    // Define a mock 3 phase bridge
    hwbridge::MOCK_BRIDGE_3PHASE mock_bridge_3phase_;

    // Define the control loop object
    Brushless6StepControlLoop brushless_6step_control_loop_;

    Brushless6StepControlLoopTest(hwbridge::BldcRotorSectorSensor* rotor_sensor = nullptr)
        : mock_clock_(), mock_bridge_3phase_(), brushless_6step_control_loop_(mock_bridge_3phase_, mock_clock_, rotor_sensor) {}

    // get desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState get_desired_state(utime_t current_time_us, utime_t time_at_start,
                                                                                float motor_speed) {
        return brushless_6step_control_loop_.get_desired_state(current_time_us, time_at_start, motor_speed);
    }

    // get zero crossing detected
    bool get_zero_crossing_detected(const hwbridge::Bridge3Phase::bemf_voltage_t& bemf_voltage, uint8_t commutation_step) {
        return brushless_6step_control_loop_.zero_crossing_detected(bemf_voltage, commutation_step);
    }

    void generate_commutation_duty_cycles(hwbridge::Bridge3Phase::phase_command_t phase_command[3], uint8_t commutation_step,
                                          float motor_speed) {
        brushless_6step_control_loop_.generate_commutation_duty_cycles(phase_command, commutation_step, motor_speed);
    }

    utime_t calculate_average_commutation_step_delta_time(utime_t new_commutation_step_delta_time,
                                                          utime_t* average_commutation_step_delta_time_array, size_t size) {
        return brushless_6step_control_loop_.calculate_average_commutation_step_delta_time(
            new_commutation_step_delta_time, average_commutation_step_delta_time_array, size);
    }
};

static Brushless6StepControlLoop::Brushless6StepControlLoopParams default_params{
    .sensored_speed_deadband_scale = 0.1f,
    .sensorless_speed_deadband_scale = 0.1f,
    .sensorless_phase_motor_startup_sequence_time_us = 1000.0f,
    .sensorless_startup_speed = 0.0f,
    .sensorless_phase_commutation_step_time_us = 1000.0f,
    .log_zero_crossing_in_sensored_mode = false,
    .sensorless_bemf_enable_backemf_skip_overrun = false,
    .bemf_zero_crossing_timeout_us = 1000.0f,
};

// Brushless 6step control loop state machine test
// Test the get desired state function when in deadband returns stop instead of startup
TEST(Brushless6StepControlLoopTest, get_desired_state_deadband) {
    Brushless6StepControlLoopTest test;

    // Set the current time to 0
    utime_t current_time_us = 0;

    // Set the time since start to 0
    utime_t time_at_start = 0;

    // Set the motor speed to half of the deadband
    float motor_speed = default_params.sensorless_speed_deadband_scale * 0.5f;

    // init the control loop
    test.brushless_6step_control_loop_.init(&default_params);

    // Get the desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        test.get_desired_state(current_time_us, time_at_start, motor_speed);

    // Check that the desired state is stop
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP);
}

// Test the get desired state function when commanded with something outside the deadband returns startup
TEST(Brushless6StepControlLoopTest, get_desired_state_startup) {
    Brushless6StepControlLoopTest test;

    // Set the current time to 1
    utime_t current_time_us = 1;

    // Set the time since start to 0
    utime_t time_at_start = 0;

    // Set the motor speed to half of the deadband
    float motor_speed = default_params.sensorless_speed_deadband_scale * 1.5f;

    // init the control loop
    test.brushless_6step_control_loop_.init(&default_params);

    // Get the desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        test.get_desired_state(current_time_us, time_at_start, motor_speed);

    // Check that the desired state is start
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::START);
}

// Test the get desired state function, after the timeout time while in startup and commanded with a value outside the deadband,
// returns run
TEST(Brushless6StepControlLoopTest, get_desired_state_run) {
    Brushless6StepControlLoopTest test;

    // Set the current time to 1
    utime_t current_time_us = default_params.sensorless_phase_motor_startup_sequence_time_us - 1;

    // Set the time since start to 0
    utime_t time_at_start = 0;

    // Set the motor speed to half of the deadband
    float motor_speed = default_params.sensorless_speed_deadband_scale * 1.5f;

    // init the control loop
    test.brushless_6step_control_loop_.init(&default_params);

    // Get the desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        test.get_desired_state(current_time_us, time_at_start, motor_speed);

    // Check that the desired state is start
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::START);
    current_time_us = default_params.sensorless_phase_motor_startup_sequence_time_us;
    desired_state = test.get_desired_state(current_time_us, time_at_start, motor_speed);
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::RUN);
}

// Test the zero crossing detection function
TEST(Brushless6StepControlLoopTest, zero_crossing_detection) {
    Brushless6StepControlLoopTest test;
    const uint8_t sector = 4;

    // Bemf voltage struct
    hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage;
    bemf_voltage.u = 3.3f;
    bemf_voltage.v = 0.0f;
    bemf_voltage.w = 3.0f;

    // Check that the zero crossing is not detected
    EXPECT_FALSE(test.get_zero_crossing_detected(bemf_voltage, sector));

    // Check that the zero crossing is detected
    bemf_voltage.u = 3.3f;
    bemf_voltage.v = 0.0f;
    bemf_voltage.w = 1.0f;
    EXPECT_TRUE(test.get_zero_crossing_detected(bemf_voltage, sector));
}

// Test the generate commutation duty cycle function
TEST(Brushless6StepControlLoopTest, generate_commutation_duty_cycle) {
    Brushless6StepControlLoopTest test;

    // Phase command struct
    hwbridge::Bridge3Phase::phase_command_t phase_command[3];

    // Check that the phase command is correct for a motor speed of 1
    // Comm step  U low, V Z, W high
    test.generate_commutation_duty_cycles(phase_command, 2, 1.0f);
    EXPECT_FLOAT_EQ(phase_command[0].duty_cycle_high_side, 0.0f);
    EXPECT_FLOAT_EQ(phase_command[0].invert_low_side, true);
    EXPECT_FLOAT_EQ(phase_command[1].duty_cycle_high_side, 0.0f);
    EXPECT_FLOAT_EQ(phase_command[1].invert_low_side, false);
    EXPECT_FLOAT_EQ(phase_command[2].duty_cycle_high_side, 1.0f);
    EXPECT_FLOAT_EQ(phase_command[2].invert_low_side, true);
}

// Test the average commutation step delta time function
TEST(Brushless6StepControlLoopTest, average_commutation_step_delta_time) {
    Brushless6StepControlLoopTest test;

    // Array of commutation step delta times
    utime_t commutation_step_delta_time_array[3] = {1000, 2000, 3000};

    // Check that the average commutation step delta time is correct
    EXPECT_EQ(test.calculate_average_commutation_step_delta_time(4000, commutation_step_delta_time_array, 3), 2000);
}

// Ensure that if the average commutation step delta time array is empty, the average commutation step delta time is 0
TEST(Brushless6StepControlLoopTest, average_commutation_step_delta_time_empty_array) {
    Brushless6StepControlLoopTest test;

    // Array of commutation step delta times
    utime_t commutation_step_delta_time_array[3] = {0, 0, 0};

    // Check that the average commutation step delta time is correct
    EXPECT_EQ(test.calculate_average_commutation_step_delta_time(4000, commutation_step_delta_time_array, 3), 0);
}

// If a rotor sensor is provided, ensure that the state machine returns stop when less than the sensored deadband
TEST(Brushless6StepControlLoopTest, rotor_sensor_stop) {
    // Define a rotor sensor
    hwbridge::MOCK_ROTOR_SECTOR_SENSOR rotor_sensor;
    Brushless6StepControlLoopTest test(&rotor_sensor);

    // Set the current time to 1
    utime_t current_time_us = 1;

    // Set the time since start to 0
    utime_t time_at_start = 0;

    // Set the motor speed to half of the deadband
    float motor_speed = default_params.sensored_speed_deadband_scale * 0.5f;

    // init the control loop
    test.brushless_6step_control_loop_.init(&default_params);

    // Get the desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        test.get_desired_state(current_time_us, time_at_start, motor_speed);

    // Check that the desired state is stop
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP);
}

// If a rotor sensor is provided, ensure that the state machine returns run when greater than the sensored deadband
TEST(Brushless6StepControlLoopTest, rotor_sensor_run) {
    // Define a rotor sensor
    hwbridge::MOCK_ROTOR_SECTOR_SENSOR rotor_sensor;
    Brushless6StepControlLoopTest test(&rotor_sensor);

    // Set the current time to 1
    utime_t current_time_us = 1;

    // Set the time since start to 0
    utime_t time_at_start = 0;

    // Set the motor speed to half of the deadband
    float motor_speed = default_params.sensored_speed_deadband_scale * 1.5f;

    // init the control loop
    test.brushless_6step_control_loop_.init(&default_params);

    // Get the desired state
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        test.get_desired_state(current_time_us, time_at_start, motor_speed);

    // Check that the desired state is stop
    EXPECT_EQ(desired_state, Brushless6StepControlLoop::Brushless6StepControlLoopState::RUN);
}

}  // namespace control_loop