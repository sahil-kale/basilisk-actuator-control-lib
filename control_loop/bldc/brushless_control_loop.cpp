#include "brushless_control_loop.hpp"

#include "bridge_3phase.hpp"
#include "brushless_6step_commutation.hpp"
#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {

// Define the init function
void BrushlessControlLoop::init(BrushlessControlLoop::BrushlessControlLoopParams* params) {
    // Initialize the rotor position estimator
    rotor_position_estimator_.reset_estimation();

    // Set the internal params pointer
    params_ = params;

    // Reset the PID controllers
    pid_d_current_.reset();
    pid_q_current_.reset();

    // Load the default id reference
    i_d_reference_ = params_->foc_params.i_d_reference_default;

    // reset the status
    status_.reset();
}

BrushlessControlLoop::BrushlessControlLoopState BrushlessControlLoop::get_desired_state(
    float motor_speed, const BrushlessControlLoopState current_state) {
    BrushlessControlLoop::BrushlessControlLoopState desired_state = current_state;
    switch (current_state) {
        case BrushlessControlLoop::BrushlessControlLoopState::NOT_INITIALIZED: {
            if (params_ != nullptr) {
                desired_state = BrushlessControlLoop::BrushlessControlLoopState::STOP;
            }
        } break;
        case BrushlessControlLoop::BrushlessControlLoopState::STOP: {
            // if the estimator reports that it is valid, then we should start the motor
            if ((motor_speed != 0)) {
                desired_state = BrushlessControlLoop::BrushlessControlLoopState::RUN;
            }
        } break;
        case BrushlessControlLoop::BrushlessControlLoopState::RUN: {
            if (motor_speed == 0.0f) {
                desired_state = BrushlessControlLoop::BrushlessControlLoopState::STOP;
            }
        } break;
        default:
            // Unknown state
            break;
    }

    return desired_state;
}

void BrushlessControlLoop::BrushlessControlLoopStatus::reset() {
    // Reset the status
    status = ControlLoopStatus::ControlLoopBaseStatus::OK;
    error = BrushlessControlLoopError::NO_ERROR;
    warning = BrushlessControlLoopWarning::NO_WARNING;
}

void BrushlessControlLoop::BrushlessControlLoopStatus::compute_base_status() {
    // If there's an error, then set the status to error
    if (error != BrushlessControlLoopError::NO_ERROR) {
        status = ControlLoopStatus::ControlLoopBaseStatus::ERROR;
    }
    // If there's a warning, then set the status to warning
    else if (warning != BrushlessControlLoopWarning::NO_WARNING) {
        status = ControlLoopStatus::ControlLoopBaseStatus::WARNING;
    }
    // Otherwise, set the status to OK
    else {
        status = ControlLoopStatus::ControlLoopBaseStatus::OK;
    }
}

ControlLoop::ControlLoopStatus BrushlessControlLoop::run_current_control(float i_d_reference, float i_q_reference) {
    if (params_->commutation_type != BrushlessControlLoopCommutationType::FOC) {
        // Set a warning in the status
        status_.warning = BrushlessControlLoopStatus::BrushlessControlLoopWarning::CURRENT_CONTROL_NOT_SUPPORTED;
    } else {
        // Update the id reference
        i_d_reference_ = i_d_reference;
        // Now, run the FOC control loop with the speed divided by the speed to iq gain
        UNUSED(run(i_q_reference / params_->foc_params.speed_to_iq_gain));
    }

    return status_;
}

ControlLoop::ControlLoopStatus BrushlessControlLoop::run(float speed) {
    // Get the current time
    utime_t current_time_us = clock_.get_time_us();

    // Clamp the speed
    math::clamp(speed, -1.0f, 1.0f);

    hwbridge::Bridge3Phase::phase_command_t phase_commands[3] = {0, false};

    // Update the rotor position estimator
    rotor_position_estimator_.update(current_time_us);

    // Get the current state and the desired state
    BrushlessControlLoop::BrushlessControlLoopState desired_state = get_desired_state(speed, state_);

    // If the desired state is different from the current state, then we need to transition
    if (desired_state != state_) {
        switch (desired_state) {
            case BrushlessControlLoop::BrushlessControlLoopState::RUN: {
                if (params_->commutation_type == BrushlessControlLoopCommutationType::FOC) {
                    // reset the PID controllers
                    pid_d_current_.reset();
                    pid_q_current_.reset();

                    // Set the PI gains
                    // First, get the phase params
                    hwbridge::Bridge3Phase::phase_params phase_params;
                    bridge_.read_phase_params(phase_params);

                    // Set the PI gains
                    const float kp = params_->foc_params.current_control_bandwidth_rad_per_sec * phase_params.inductance;
                    const float ki = phase_params.resistance / phase_params.inductance * kp *
                                     static_cast<float>((params_->foc_params.disable_ki ==
                                                         true));  // multiplied by kp to create a series PI controller

                    pid_d_current_.set_kp(kp);
                    pid_d_current_.set_ki(ki);

                    pid_q_current_.set_kp(kp);
                    pid_q_current_.set_ki(ki);

                    // reset the rotor position estimator
                    rotor_position_estimator_.reset_estimation();
                    // Set the desired rotor angle to the current rotor angle
                    rotor_position_estimator_.get_rotor_position(desired_rotor_angle_open_loop_);
                }
            } break;
            case BrushlessControlLoop::BrushlessControlLoopState::NOT_INITIALIZED: {
                // return an error
                status_.error = BrushlessControlLoopStatus::BrushlessControlLoopError::PARAMS_NOT_SET;
            } break;
            case BrushlessControlLoop::BrushlessControlLoopState::STOP:
            default:
                break;
        }
        state_ = desired_state;
    }

    // Run the state machine
    control_loop_type_ = get_desired_control_loop_type(rotor_position_estimator_.is_estimation_valid());
    switch (state_) {
        case BrushlessControlLoop::BrushlessControlLoopState::STOP:
            break;

        case BrushlessControlLoop::BrushlessControlLoopState::RUN: {
            switch (params_->commutation_type) {
                case BrushlessControlLoopCommutationType::FOC: {
                    run_foc(speed, current_time_us, last_run_time_, phase_commands);
                } break;
                case BrushlessControlLoopCommutationType::TRAPEZOIDAL: {
                    run_trap(speed, phase_commands);
                } break;
                default:
                    break;
            }
        } break;

        default:
            break;
    }

    // Set the duty cycles
    this->bridge_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);

    last_run_time_ = current_time_us;

    // Compute the base status
    status_.compute_base_status();

    return status_;
}

void BrushlessControlLoop::run_foc(float speed, utime_t current_time_us, utime_t last_run_time_us,
                                   hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    // Get the bus voltage
    float bus_voltage = 0.0f;
    // TODO: ERROR CHECKING!!
    bridge_.read_bus_voltage(bus_voltage);

    switch (control_loop_type_) {
        case BrushlessControlLoopType::OPEN_LOOP: {
            V_quadrature_ = speed * bus_voltage;
            V_direct_ = 0.0f;  // TODO: add param for open-loop direct voltage

            // increment the rotor position by the speed multiplied by the time since the last run
            desired_rotor_angle_open_loop_ += params_->open_loop_full_speed_theta_velocity * speed *
                                              (float)(current_time_us - last_run_time_) /
                                              basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;
            // Wrap the rotor position around 0 and 2pi
            math::wraparound(desired_rotor_angle_open_loop_, 0.0f, float(2.0f * M_PI));

            rotor_position_ = desired_rotor_angle_open_loop_;

        } break;
        case BrushlessControlLoopType::CLOSED_LOOP: {
            // Get the FOC current
            hwbridge::Bridge3Phase::phase_current_t phase_currents;
            bridge_.read_current(phase_currents);

            rotor_position_estimator_.get_rotor_position(rotor_position_);

            // Do a Clarke transform
            math::clarke_transform_result_t clarke_transform =
                math::clarke_transform(phase_currents.u, phase_currents.v, phase_currents.w);

            // Do a Park transform
            math::park_transform_result_t park_transform_currents =
                math::park_transform(clarke_transform.alpha, clarke_transform.beta, rotor_position_);

            // Determine the tau for the LPF for the current controller
            const float tau = math::determine_tau_from_f_c(params_->foc_params.current_lpf_fc);
            const float dt = clock_.get_dt_s(current_time_us, last_run_time_us);

            // LPF the currents
            i_direct_ = math::low_pass_filter(park_transform_currents.d, i_direct_, tau, dt);
            i_quadrature_ = math::low_pass_filter(park_transform_currents.q, i_quadrature_, tau, dt);

            i_direct_ = park_transform_currents.d;
            i_quadrature_ = park_transform_currents.q;

            // Run the PI controller
            // The below hack for speed is kinda hacky and should be reverted lol
            V_quadrature_ += pid_q_current_.calculate(i_quadrature_, speed * params_->foc_params.speed_to_iq_gain);
            V_direct_ += pid_d_current_.calculate(i_direct_, i_d_reference_);
        } break;
        default:
            break;
    }
    // Limit the Vd and Vq by first calculating the modulus of the vector
    const float V_modulus = sqrtf(V_direct_ * V_direct_ + V_quadrature_ * V_quadrature_);
    // If the modulus is greater than the bus voltage, then we need to scale the voltage vector
    if (V_modulus > bus_voltage) {
        // Scale the voltage vector
        V_direct_ = V_direct_ * bus_voltage / V_modulus;
        V_quadrature_ = V_quadrature_ * bus_voltage / V_modulus;
    }

    // Determine the appropriate duty cycles for the inverter
    determine_inverter_duty_cycles_foc(rotor_position_, V_direct_, V_quadrature_, bus_voltage,
                                       params_->foc_params.pwm_control_type, phase_commands[0], phase_commands[1],
                                       phase_commands[2]);
}

void BrushlessControlLoop::run_trap(float speed, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    // Get the rotor position
    rotor_position_estimator_.get_rotor_position(rotor_position_);

    // Get the commutation step
    Bldc6StepCommutationTypes::commutation_step_t current_commutation_step =
        Bldc6StepCommutationTypes::determine_commutation_step_from_theta(rotor_position_);

    // Determine the duty cycles for the inverter
    determine_inverter_duty_cycles_trap(phase_commands, current_commutation_step, speed);
}

void BrushlessControlLoop::determine_inverter_duty_cycles_foc(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                                              BrushlessControlLoop::BrushlessFocPwmControlType pwm_control_type,
                                                              hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                              hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                              hwbridge::Bridge3Phase::phase_command_t& phase_command_w) {
    switch (pwm_control_type) {
        case BrushlessControlLoop::BrushlessFocPwmControlType::SPACE_VECTOR: {
            math::svpwm_duty_cycle_t duty_cycles = math::svpwm(Vdirect, Vquadrature, theta, bus_voltage);
            duty_cycle_u_h_ = duty_cycles.dutyCycleU;
            duty_cycle_v_h_ = duty_cycles.dutyCycleV;
            duty_cycle_w_h_ = duty_cycles.dutyCycleW;
        } break;
        case BrushlessControlLoop::BrushlessFocPwmControlType::SINE: {
            // Do an inverse Park transform
            math::inverse_park_transform_result_t inverse_park_transform =
                math::inverse_park_transform(Vdirect, Vquadrature, theta);

            V_alpha_ = inverse_park_transform.alpha;
            V_beta_ = inverse_park_transform.beta;

            // Do an inverse clarke transform
            math::inverse_clarke_transform_result_t inverse_clarke_transform =
                math::inverse_clarke_transform(inverse_park_transform.alpha, inverse_park_transform.beta);

            // load the results into the phase commands
            duty_cycle_u_h_ = inverse_clarke_transform.a / bus_voltage;
            duty_cycle_v_h_ = inverse_clarke_transform.b / bus_voltage;
            duty_cycle_w_h_ = inverse_clarke_transform.c / bus_voltage;

            // Duty cycles can be between -1 and 1, and those should linearly map to 0 -> 1
            duty_cycle_u_h_ = (duty_cycle_u_h_ + this->MAX_MOTOR_SPEED) / (this->MAX_MOTOR_SPEED * 2.0f);
            duty_cycle_v_h_ = (duty_cycle_v_h_ + this->MAX_MOTOR_SPEED) / (this->MAX_MOTOR_SPEED * 2.0f);
            duty_cycle_w_h_ = (duty_cycle_w_h_ + this->MAX_MOTOR_SPEED) / (this->MAX_MOTOR_SPEED * 2.0f);
        } break;
        default:
            // Set the duty cycles to 0
            duty_cycle_u_h_ = 0.0f;
            duty_cycle_v_h_ = 0.0f;
            duty_cycle_w_h_ = 0.0f;
            break;
    }

    // No matter what, the duty cycles should be between 0 and 1
    math::clamp(duty_cycle_u_h_, 0.0f, 1.0f);
    math::clamp(duty_cycle_v_h_, 0.0f, 1.0f);
    math::clamp(duty_cycle_w_h_, 0.0f, 1.0f);

    // Set the duty cycles
    phase_command_u.duty_cycle_high_side = duty_cycle_u_h_;
    phase_command_u.invert_low_side = true;
    phase_command_v.duty_cycle_high_side = duty_cycle_v_h_;
    phase_command_v.invert_low_side = true;
    phase_command_w.duty_cycle_high_side = duty_cycle_w_h_;
    phase_command_w.invert_low_side = true;
}

BrushlessControlLoop::BrushlessControlLoopType BrushlessControlLoop::get_desired_control_loop_type(bool is_estimator_valid) {
    BrushlessControlLoop::BrushlessControlLoopType desired_control_loop_type =
        BrushlessControlLoop::BrushlessControlLoopType::OPEN_LOOP;
    if (is_estimator_valid) {
        desired_control_loop_type = BrushlessControlLoop::BrushlessControlLoopType::CLOSED_LOOP;
    }
    return desired_control_loop_type;
}

void BrushlessControlLoop::determine_inverter_duty_cycles_trap(
    hwbridge::Bridge3Phase::phase_command_t phase_command[3],
    Bldc6StepCommutationTypes::commutation_step_t current_commutation_step, float motor_speed) {
    for (int i = 0; i < 3; i++) {
        if (current_commutation_step.signals[i] == Bldc6StepCommutationTypes::CommutationSignal::HIGH) {
            phase_command[i].duty_cycle_high_side = fabs(motor_speed);
            phase_command[i].invert_low_side = true;
        } else if (current_commutation_step.signals[i] == Bldc6StepCommutationTypes::CommutationSignal::LOW) {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].invert_low_side = true;
        } else {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].invert_low_side = false;
        }
    }
}

}  // namespace control_loop
