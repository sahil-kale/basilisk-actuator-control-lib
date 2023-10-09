#include "brushless_foc_control_loop.hpp"

#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"

namespace control_loop {

// Define the init function
void BrushlessFocControlLoop::init(BrushlessFocControlLoop::BrushlessFocControLoopParams* params) {
    // Initialize the rotor position estimator
    rotor_position_estimator_.reset_estimation();

    // Set the internal params pointer
    params_ = params;

    // Initialize the PID controllers
    pid_d_current_.set_kd(params_->kd_d_current);
    pid_d_current_.set_ki(params_->ki_d_current);
    pid_d_current_.set_kp(params_->kp_d_current);

    pid_q_current_.set_kd(params_->kd_q_current);
    pid_q_current_.set_ki(params_->ki_q_current);
    pid_q_current_.set_kp(params_->kp_q_current);

    // Reset the PID controllers
    pid_d_current_.reset();
    pid_q_current_.reset();
}

BrushlessFocControlLoop::BrushlessFocControlLoopState BrushlessFocControlLoop::get_desired_state(
    float motor_speed, const BrushlessFocControlLoopState current_state) {
    BrushlessFocControlLoop::BrushlessFocControlLoopState desired_state = current_state;
    switch (current_state) {
        case BrushlessFocControlLoop::BrushlessFocControlLoopState::NOT_INITIALIZED: {
            if (params_ != nullptr) {
                desired_state = BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP;
            }
        } break;
        case BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP: {
            // if the estimator reports that it is valid, then we should start the motor
            if ((motor_speed != 0)) {
                desired_state = BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN;
            }
        } break;
        case BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN: {
            if (motor_speed == 0.0f) {
                desired_state = BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP;
            }
        } break;
        default:
            // Unknown state
            break;
    }

    return desired_state;
}

void BrushlessFocControlLoop::run(float speed) {
    // Get the current time
    utime_t current_time_us = clock_.get_time_us();

    hwbridge::Bridge3Phase::phase_command_t phase_commands[3] = {0, false};

    // Update the rotor position estimator
    rotor_position_estimator_.update(current_time_us);

    // Get the current state and the desired state
    BrushlessFocControlLoop::BrushlessFocControlLoopState desired_state = get_desired_state(speed, state_);

    // If the desired state is different from the current state, then we need to transition
    if (desired_state != state_) {
        switch (desired_state) {
            case BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN: {
                // reset the PID controllers
                pid_d_current_.reset();
                pid_q_current_.reset();
                // reset the rotor position estimator
                rotor_position_estimator_.reset_estimation();
                // Set the desired rotor angle to the current rotor angle
                rotor_position_estimator_.get_rotor_position(desired_rotor_angle_open_loop_);
            } break;
            case BrushlessFocControlLoop::BrushlessFocControlLoopState::NOT_INITIALIZED:
            case BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP:
            default:
                break;
        }
        state_ = desired_state;
    }

    // Run the state machine
    switch (state_) {
        case BrushlessFocControlLoop::BrushlessFocControlLoopState::STOP:
            break;

        case BrushlessFocControlLoop::BrushlessFocControlLoopState::RUN: {
            // Get the bus voltage
            float bus_voltage = 0.0f;
            // TODO: ERROR CHECKING!!
            bridge_.read_bus_voltage(bus_voltage);

            // Determine the control loop type
            const BrushlessFocControlLoopType desired_control_loop_type =
                get_desired_control_loop_type(rotor_position_estimator_.is_estimation_valid());
            switch (desired_control_loop_type) {
                case BrushlessFocControlLoopType::OPEN_LOOP: {
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
                case BrushlessFocControlLoopType::CLOSED_LOOP: {
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

                    i_direct_ = park_transform_currents.d;
                    i_quadrature_ = park_transform_currents.q;

                    // Run the PI controller
                    // The below hack for speed is kinda hacky and should be reverted lol
                    V_quadrature_ += pid_q_current_.calculate(i_quadrature_, speed * params_->speed_to_iq_gain);
                    V_direct_ += pid_d_current_.calculate(i_direct_, params_->i_d_reference);
                } break;
                default:
                    break;
            }
            // Limit the Vd and Vq
            math::clamp(V_direct_, -bus_voltage, bus_voltage);
            math::clamp(V_quadrature_, -bus_voltage, bus_voltage);

            // Determine the appropriate duty cycles for the inverter
            determine_inverter_duty_cycles(rotor_position_, V_direct_, V_quadrature_, bus_voltage, params_->pwm_control_type,
                                           phase_commands[0], phase_commands[1], phase_commands[2]);
        } break;

        default:
            break;
    }

    // Set the duty cycles
    this->bridge_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);

    last_run_time_ = current_time_us;
}

void BrushlessFocControlLoop::determine_inverter_duty_cycles(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                                             BrushlessFocControlLoop::BrushlessFocPwmControlType pwm_control_type,
                                                             hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                             hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                             hwbridge::Bridge3Phase::phase_command_t& phase_command_w) {
    switch (pwm_control_type) {
        case BrushlessFocControlLoop::BrushlessFocPwmControlType::SPACE_VECTOR: {
            math::svpwm_duty_cycle_t duty_cycles = math::svpwm(Vdirect, Vquadrature, theta, bus_voltage);
            duty_cycle_u_h_ = duty_cycles.dutyCycleU;
            duty_cycle_v_h_ = duty_cycles.dutyCycleV;
            duty_cycle_w_h_ = duty_cycles.dutyCycleW;
        } break;
        case BrushlessFocControlLoop::BrushlessFocPwmControlType::SINE: {
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

    // Set the duty cycles
    phase_command_u.duty_cycle_high_side = duty_cycle_u_h_;
    phase_command_u.invert_low_side = true;
    phase_command_v.duty_cycle_high_side = duty_cycle_v_h_;
    phase_command_v.invert_low_side = true;
    phase_command_w.duty_cycle_high_side = duty_cycle_w_h_;
    phase_command_w.invert_low_side = true;
}

BrushlessFocControlLoop::BrushlessFocControlLoopType BrushlessFocControlLoop::get_desired_control_loop_type(
    bool is_estimator_valid) {
    BrushlessFocControlLoop::BrushlessFocControlLoopType desired_control_loop_type =
        BrushlessFocControlLoop::BrushlessFocControlLoopType::OPEN_LOOP;
    if (is_estimator_valid) {
        desired_control_loop_type = BrushlessFocControlLoop::BrushlessFocControlLoopType::CLOSED_LOOP;
    }
    return desired_control_loop_type;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::reset_estimation() {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position_ = 0.0f;
    velocity_ = 0.0f;
    compensated_velocity_ = 0.0f;
    raw_hall_angle_ = 0.0f;
    time_at_last_hall_update_ = 0;
    time_update_last_called_ = 0;
    number_of_hall_updates_ = 0;

    // Update the rotor position with the sector sensor
    uint8_t sector = 0;
    ret = sector_sensor_.get_sector(sector);
    if (ret == APP_HAL_OK) {
        rotor_position_ = sector * 2.0 * M_PI / 6.0f;
    }

    return ret;
}

bool BldcElectricalRotorPositionEstimatorFromHall::is_estimation_valid() {
    bool ret = false;
    if (params_ != nullptr) {
        bool num_hall_updates_to_start = (number_of_hall_updates_ > params_->num_hall_updates_to_start);
        // Also make the return conditional if the position estimate no greater than the param for tolerance
        bool rotor_position_tolerance = (fabs(rotor_position_ - raw_hall_angle_) < params_->max_estimate_angle_overrun);
        ret = num_hall_updates_to_start && rotor_position_tolerance;
    }
    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::init(
    BldcElectricalRotorPositionEstimatorFromHallParams_t* params) {
    app_hal_status_E ret = APP_HAL_OK;

    // Set the internal params pointer
    params_ = params;

    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::update(utime_t time) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        if (params_ == nullptr) {
            ret = APP_HAL_ERROR;
            break;
        }

        uint8_t sector = 0;

        ret = sector_sensor_.get_sector(sector);

        if (ret != APP_HAL_OK) {
            break;
        }

        float raw_hall_angle = sector * 2 * M_PI / 6;
        // We should not update velocity estimations if the raw hall angle is the same as the previous one
        if (fabs(raw_hall_angle - raw_hall_angle_) > 0.001f) {
            // Calculate velocity with a simple differentiation
            const float time_delta_since_hall_update =
                (float)(time - time_at_last_hall_update_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;

            float raw_hall_angle_diff = raw_hall_angle - raw_hall_angle_;

            // Account for over/underflow
            // NOTE: if the rotor delta theta is greater than pi radians, then this detection will not work
            math::wraparound(raw_hall_angle_diff, static_cast<float>(-M_PI), static_cast<float>(M_PI));

            velocity_ = raw_hall_angle_diff / time_delta_since_hall_update;

            // Calculate a compensated velocity to account for position error and smoothly compensate for it
            compensated_velocity_ = velocity_;
            // TODO: implement compensated_velocity_ = velocity_ * (1 - ((rotor_position_ - raw_hall_angle_diff) /
            // raw_hall_angle_diff));

            rotor_position_ = raw_hall_angle;
            this->raw_hall_angle_ = raw_hall_angle;
            time_at_last_hall_update_ = time;
            number_of_hall_updates_++;
        }

        const float current_measurement_period =
            (float)(time - time_update_last_called_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;
        // Update the rotor position with the velocity estimate
        rotor_position_ += compensated_velocity_ * current_measurement_period;

        // Implement a wraparound
        math::wraparound(rotor_position_, 0.0f, float(2.0f * M_PI));

        time_update_last_called_ = time;
    } while (false);
    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::get_rotor_position(float& rotor_position) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position = rotor_position_;

    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::get_rotor_velocity(float& rotor_velocity) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_velocity = compensated_velocity_;

    return ret;
}

}  // namespace control_loop
