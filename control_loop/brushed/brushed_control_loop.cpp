#include "brushed_control_loop.hpp"

namespace control_loop {

BrushedControlLoop::BrushedControlLoop() {}

void BrushedControlLoop::run(float speed) {
    // Get the current time
    // First check if clock is valid. If not, return early
    if (!clock_) {
        return;
    }

    utime_t current_time_us = clock_->get_time_us();

    // Get the motor speed from the system manager
    float motor_speed = speed;

    // Get the brake mode from the params
    if (params_ == nullptr) {
        return;
    }
    bool brake_mode = params_->brake_mode_enabled;

    // Compute the motor speed outputs
    h_bridge_motor_speed_outputs_t motor_speed_outputs = compute_motor_speed_outputs(motor_speed, brake_mode, current_time_us);

    // Set the motor speed outputs
    // TODO: implement this with 4 timers that can be registered
    (void)motor_speed_outputs;
}

BrushedControlLoop::h_bridge_motor_speed_outputs_t BrushedControlLoop::compute_motor_speed_outputs(float motor_speed,
                                                                                                   bool brake_mode,
                                                                                                   utime_t current_time_us) {
    h_bridge_motor_speed_outputs_t motor_speed_outputs = {0.0f, 0.0f, 0.0f, 0.0f};

    if (params_ == nullptr) {
        return motor_speed_outputs;
    }

    // Check if the motor speed is 0.0f
    if (motor_speed == 0.0f) {
        // Check if brake mode is enabled
        if (brake_mode) {
            // Set the duty cycle of the low pins to 1.0f and the high pins to 0.0f
            motor_speed_outputs.DC_A_HIGH = 0.0f;
            motor_speed_outputs.DC_A_LOW = 1.0f;
            motor_speed_outputs.DC_B_HIGH = 0.0f;
            motor_speed_outputs.DC_B_LOW = 1.0f;

        } else {
            // Set the duty cycle of all pins to 0.0f
            motor_speed_outputs.DC_A_HIGH = 0.0f;
            motor_speed_outputs.DC_A_LOW = 0.0f;
            motor_speed_outputs.DC_B_HIGH = 0.0f;
            motor_speed_outputs.DC_B_LOW = 0.0f;
        }
    } else if (motor_speed >= 0.0f) {
        // A high and B low should be set to a scale of absolute value of motor speed from 0.0f to 1.0f
        // using the max motor speed constant
        motor_speed_outputs.DC_A_HIGH = motor_speed / MAX_MOTOR_SPEED;
        motor_speed_outputs.DC_A_LOW = 0.0f;
        motor_speed_outputs.DC_B_HIGH = 0.0f;
        motor_speed_outputs.DC_B_LOW = motor_speed / MAX_MOTOR_SPEED;
    } else if (motor_speed < 0.0f) {
        // A low and B high should be set to a scale of absolute value of motor speed from 0.0f to 1.0f
        // using the max motor speed constant
        motor_speed_outputs.DC_A_HIGH = 0.0f;
        motor_speed_outputs.DC_A_LOW = -motor_speed / MAX_MOTOR_SPEED;
        motor_speed_outputs.DC_B_HIGH = -motor_speed / MAX_MOTOR_SPEED;
        motor_speed_outputs.DC_B_LOW = 0.0f;
    }

    // Dead time insertion- if the signs of the last successful motor speed and the current motor speed are different
    // and the time is less than the last speed change time plus the dead time constant, then set the duty cycle of all
    // pins to 0.0f

    // First check if the signs are different
    if ((motor_speed * last_motor_speed_ < 0.0f)) {
        // Update the last changed time
        last_speed_dir_change_time_us_ = current_time_us;
    }

    // Check if the time is less than the last speed change time plus the dead time constant
    if (current_time_us < params_->h_bridge_dead_time_us + last_speed_dir_change_time_us_) {
        // Set the duty cycle of all pins to 0.0f
        motor_speed_outputs.DC_A_HIGH = 0.0f;
        motor_speed_outputs.DC_A_LOW = 0.0f;
        motor_speed_outputs.DC_B_HIGH = 0.0f;
        motor_speed_outputs.DC_B_LOW = 0.0f;
    }

    last_motor_speed_ = motor_speed;

    return motor_speed_outputs;
}

void BrushedControlLoop::init(BrushedControlLoopParams* params) {
    params_ = params;
    last_speed_dir_change_time_us_ = 0;
}

}  // namespace control_loop