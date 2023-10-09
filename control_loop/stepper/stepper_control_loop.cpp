#include "stepper_control_loop.hpp"

#include "math.h"

namespace control_loop {

// Run the stepper control loop
void StepperControlLoop::run(float speed) {
    // Get the current time
    utime_t current_time = clock_.get_time_us();
    // Edge case: If this is the first time this function is called, set the previous time to the current time
    if (previous_time_ == 0) {
        previous_time_ = current_time;
    }
    // Get the desired speed from the system manager
    float desired_speed = speed;

    // Determine the time delta since the last time this function was called
    float time_delta = (current_time - previous_time_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;
    float desired_angle = previous_angle_;

    if (params_->stepper_motor_simple_switcher_enabled) {
        // Call the determineElectricalAngleSimple function to determine the electrical angle of the motor
        desired_angle = determineElectricalAngleSimple(time_delta, desired_speed, previous_angle_);
    } else {
        // Determine the electrical angle of the motor
        desired_angle = determineElectricalAngleSimple(time_delta, desired_speed, previous_angle_);
        // Update the previous time and angle
        previous_time_ = clock_.get_time_us();
        previous_angle_ = desired_angle;
    }

    // Determine the A and B current scalars based on the electrical angle
    std::pair<float, float> current_scalars = determineCurrentSetpointScalars(desired_angle);

    if (params_->stepper_motor_disable_current_pid) {
        // Set the current setpoints of the motors
        motor_a_.run(params_->stepper_motor_current_to_pwm_duty_cycle_slope * current_scalars.first);
        motor_b_.run(params_->stepper_motor_current_to_pwm_duty_cycle_slope * current_scalars.second);
    }
}

// Determine the electrical angle of the motor
float StepperControlLoop::determineElectricalAngle(float time_delta, float desired_speed, float previous_angle) {
    // determine the angle based on the desired speed and the time delta
    // A step is 360 degrees / 4 blocks = 90 degrees
    float angle = previous_angle + (desired_speed * time_delta) * 360.0 / 4;
    // If the angle is greater than 360, subtract 360
    if (angle > 360.0f) {
        angle -= 360.0f;
    }
    // If the angle is less than 0, add 360
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

float StepperControlLoop::determineElectricalAngleSimple(float time_delta, float desired_speed, float previous_angle) {
    // call the determineElectricalAngle function
    float angle = determineElectricalAngle(time_delta, desired_speed, previous_angle);
    // calculate the difference between the angle and the previous angle
    float angle_diff = angle - previous_angle;
    // If the angle difference is greater than 90 degrees, return that angle. Otherwise, return the previous angle
    if (fabs(angle_diff) > 90.0f) {
        previous_time_ = clock_.get_time_us();
        previous_angle_ = angle;
        return angle;
    } else {
        return previous_angle;
    }
}

std::pair<float, float> StepperControlLoop::determineCurrentSetpointScalars(float electrical_angle) {
    // Determine the A and B current scalars based on the electrical angle
    // The A current scalar is the cosine of the electrical angle
    // The B current scalar is the sine of the electrical angle
    float a_current_scalar = cos(electrical_angle * M_PI / 180.0f);
    float b_current_scalar = sin(electrical_angle * M_PI / 180.0f);
    return std::make_pair(a_current_scalar, b_current_scalar);
}

}  // namespace control_loop