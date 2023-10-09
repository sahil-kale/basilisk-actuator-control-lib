#include "brushless_6step_control_loop.hpp"

#include <math.h>

namespace control_loop {

Brushless6StepControlLoop::Brushless6StepControlLoopState Brushless6StepControlLoop::get_desired_state(utime_t current_time_us,
                                                                                                       utime_t time_at_start,
                                                                                                       float motor_speed) {
    Brushless6StepControlLoop::Brushless6StepControlLoopState return_state =
        Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP;

    if (this->rotor_sensor_) {
        if (fabs(motor_speed) < params_->sensored_speed_deadband_scale) {
            return_state = Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP;
        } else {
            return_state = Brushless6StepControlLoop::Brushless6StepControlLoopState::RUN;
        }
    } else {
        if (fabs(motor_speed) < params_->sensorless_speed_deadband_scale) {
            return_state = Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP;
        } else if (current_time_us - time_at_start < params_->sensorless_phase_motor_startup_sequence_time_us) {
            return_state = Brushless6StepControlLoop::Brushless6StepControlLoopState::START;
        } else {
            return_state = Brushless6StepControlLoop::Brushless6StepControlLoopState::RUN;
        }
    }
    // When stopped, update the time_at_start variable
    if (return_state == Brushless6StepControlLoop::Brushless6StepControlLoopState::STOP) {
        time_at_start_ = time_at_start;
        time_at_start_ = current_time_us;
    }
    return return_state;
}

bool Brushless6StepControlLoop::zero_crossing_detected(const hwbridge::Bridge3Phase::bemf_voltage_t& bemf_voltage,
                                                       uint8_t commutation_step) {
    const commutation_step_t current_commutation_step = commutation_steps[commutation_step];
    float phase_sum = 0.0f;
    CommutationSignal zero_crossing_signal = CommutationSignal::Z_RISING;
    float undriven_phase_voltage = 0.0f;

    float bemf_voltages[hwbridge::Bridge3Phase::NUM_PHASES] = {bemf_voltage.u, bemf_voltage.v, bemf_voltage.w};

    for (uint8_t i = 0; i < hwbridge::Bridge3Phase::NUM_PHASES; i++) {
        if ((current_commutation_step.signals[i] != CommutationSignal::Z_FALLING) &&
            (current_commutation_step.signals[i] != CommutationSignal::Z_RISING)) {
            phase_sum += bemf_voltages[i];
        } else {
            zero_crossing_signal = current_commutation_step.signals[i];
            undriven_phase_voltage = bemf_voltages[i];
        }
    }

    float zero_crossing_threshold = phase_sum / 2.0f;  // NOTE: This requires the bemf voltage to run when the PWM is ON
    bool return_value = false;
    if (zero_crossing_signal == CommutationSignal::Z_RISING) {
        if (undriven_phase_voltage > zero_crossing_threshold) {
            return_value = true;
        }
    } else {
        if (undriven_phase_voltage < zero_crossing_threshold) {
            return_value = true;
        }
    }

    return return_value;
}

void Brushless6StepControlLoop::init(Brushless6StepControlLoopParams* params) { params_ = params; }

void Brushless6StepControlLoop::run(float speed) {
    if (params_ == nullptr) {
        return;
    }

    hwbridge::Bridge3Phase::phase_command_t phase_commands[3] = {0, 0};

    // Get the desired motor speed from the system manager
    float desired_motor_speed = speed;

    // Get the current time
    utime_t current_time_us = this->clock_.get_time_us();

    // Get the desired state of the control loop
    Brushless6StepControlLoop::Brushless6StepControlLoopState desired_state =
        get_desired_state(current_time_us, time_at_start_, desired_motor_speed);

    if (desired_state != state_) {
        switch (desired_state) {
            case Brushless6StepControlLoopState::STOP: {
                commutation_step_ = 0;
                motor_speed_ = 0;
                break;
            }
            case Brushless6StepControlLoopState::START: {
                last_commutation_step_switch_time_ = current_time_us;
                break;
            }
            case Brushless6StepControlLoopState::RUN: {
                break;
            }
            default: {
                break;
            }
        }
        state_ = desired_state;
    }

    // Switch case on states:
    switch (state_) {
        case Brushless6StepControlLoopState::STOP: {
            for (int i = 0; i < hwbridge::Bridge3Phase::NUM_PHASES; i++) {
                phase_commands[i].duty_cycle_high_side = 0.0f;
                phase_commands[i].invert_low_side = false;
            }
            this->zero_crossing_time_ = 0;
            break;
        }
        case Brushless6StepControlLoopState::START: {
            // This defines the startup sequence for the motor
            // Use the deadband scale to determine the speed to run the motor at
            float startup_motor_speed = params_->sensorless_startup_speed;
            // Check if we should switch to the next commutation step
            if (current_time_us - last_commutation_step_switch_time_ > params_->sensorless_phase_commutation_step_time_us) {
                // Switch to the next commutation step
                commutation_step_++;
                if (commutation_step_ > 5) {
                    commutation_step_ = 0;
                }
                last_commutation_step_switch_time_ = current_time_us;
            }

            // Compute the phase commands
            generate_commutation_duty_cycles(phase_commands, commutation_step_, startup_motor_speed);
            break;
        }
        case Brushless6StepControlLoopState::RUN: {
            motor_speed_ = desired_motor_speed;
            // Check first if a rotor is defined
            if (this->rotor_sensor_) {
                uint8_t sector = 0;
                // TODO: Handle failure if HAL fails
                rotor_sensor_->get_sector(sector);
                if (sector != commutation_step_) {
                    this->update_average_commutation_step_delta_time(current_time_us);
                    commutation_step_ = sector;
                    zero_crossing_time_ = 0;
                }

                // Log zerocrossings if enabled in param server
                if (params_->log_zero_crossing_in_sensored_mode) {
                    // Get the bemf voltage
                    hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage;
                    this->motor_.read_bemf(bemf_voltage);
                    // If the desired motor speed is negative, we need to swap the v and w phase bemf readings
                    if (motor_speed_ < 0) {
                        float temp = bemf_voltage.v;
                        bemf_voltage.v = bemf_voltage.w;
                        bemf_voltage.w = temp;
                    }
                    // Check if we have detected a zero crossing. Only update the variable if the ZCT is 0
                    if (zero_crossing_detected(bemf_voltage, commutation_step_) && zero_crossing_time_ == 0 &&
                        (current_time_us - last_commutation_step_switch_time_) > params_->bemf_zero_crossing_timeout_us) {
                        zero_crossing_time_ = current_time_us;
                    }
                }

            } else {
                // Check if we should switch to the next commutation step
                // First, check if we have detected a zero crossing on the bemf voltage
                if (zero_crossing_time_ == 0) {
                    // Get the bemf voltage
                    hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage;
                    this->motor_.read_bemf(bemf_voltage);
                    // If the desired motor speed is negative, we need to swap the v and w phase bemf readings
                    if (motor_speed_ < 0) {
                        float temp = bemf_voltage.v;
                        bemf_voltage.v = bemf_voltage.w;
                        bemf_voltage.w = temp;
                    }
                    // Check if we have detected a zero crossing
                    // If we don't detect a zero crossing, look at the average commutation step time to see if we should switch
                    if (zero_crossing_detected(bemf_voltage, commutation_step_) &&
                        (current_time_us - last_commutation_step_switch_time_) > params_->bemf_zero_crossing_timeout_us) {
                        zero_crossing_time_ = current_time_us;
                    } else if (average_commutation_step_delta_time_ != 0 &&
                               current_time_us - last_commutation_step_switch_time_ > average_commutation_step_delta_time_ &&
                               params_->sensorless_bemf_enable_backemf_skip_overrun) {
                        // Switch to the next commutation step
                        commutation_step_switch_sensorless(current_time_us);
                    }
                } else {
                    // Check if we should switch to the next commutation step
                    if (current_time_us >
                        2 * (zero_crossing_time_ - last_commutation_step_switch_time_) + last_commutation_step_switch_time_) {
                        // Switch to the next commutation step
                        commutation_step_switch_sensorless(current_time_us);
                    }
                }
            }

            // Compute the phase commands
            generate_commutation_duty_cycles(phase_commands, commutation_step_, motor_speed_);

            break;
        }
    }

    // Send the phase commands to the bridge
    // if the desired motor speed is negative, we need to swap the u, v, and w phase commands
    if (motor_speed_ < 0) {
        hwbridge::Bridge3Phase::phase_command_t temp = phase_commands[2];
        phase_commands[2] = phase_commands[1];
        phase_commands[1] = phase_commands[0];
        phase_commands[0] = temp;
    }
    this->motor_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);
}

void Brushless6StepControlLoop::generate_commutation_duty_cycles(hwbridge::Bridge3Phase::phase_command_t phase_command[3],
                                                                 uint8_t commutation_step, float motor_speed) {
    const commutation_step_t current_commutation_step = commutation_steps[commutation_step];
    for (int i = 0; i < 3; i++) {
        if (current_commutation_step.signals[i] == CommutationSignal::HIGH) {
            phase_command[i].duty_cycle_high_side = fabs(motor_speed);
            phase_command[i].invert_low_side = true;
        } else if (current_commutation_step.signals[i] == CommutationSignal::LOW) {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].invert_low_side = true;
        } else {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].invert_low_side = false;
        }
    }
}

void Brushless6StepControlLoop::commutation_step_switch_sensorless(utime_t current_time_us) {
    // Switch to the next commutation step
    commutation_step_++;
    // update the average commutation step delta time
    this->update_average_commutation_step_delta_time(current_time_us);
    if (commutation_step_ > 5) {
        commutation_step_ = 0;
    }
    // Reset the zero crossing time
    zero_crossing_time_ = 0;
}

utime_t Brushless6StepControlLoop::calculate_average_commutation_step_delta_time(
    utime_t new_commutation_step_delta_time, utime_t* average_commutation_step_delta_time_array, size_t size) {
    // Calculate the average commutation step delta time
    utime_t average_commutation_step_delta_time = 0;
    for (size_t i = 0; i < size; i++) {
        average_commutation_step_delta_time += average_commutation_step_delta_time_array[i];
    }
    average_commutation_step_delta_time /= size;

    // Shift the array
    for (size_t i = size - 1; i > 0; i--) {
        average_commutation_step_delta_time_array[i] = average_commutation_step_delta_time_array[i - 1];
    }
    average_commutation_step_delta_time_array[0] = new_commutation_step_delta_time;

    // Check that the last element is not zero
    if (average_commutation_step_delta_time_array[size - 1] == 0) {
        average_commutation_step_delta_time = 0;
    }

    return average_commutation_step_delta_time;
}

}  // namespace control_loop
