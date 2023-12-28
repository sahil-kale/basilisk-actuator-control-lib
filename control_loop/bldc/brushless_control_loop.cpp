#include "brushless_control_loop.hpp"

#include "bridge_3phase.hpp"
#include "brushless_6step_commutation.hpp"
#include "brushless_foc.hpp"
#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {

// Define the init function
void BrushlessControlLoop::init(BrushlessControlLoop::BrushlessControlLoopParams* params) {
    // Initialize the rotor position estimator
    primary_rotor_position_estimator_.reset_estimation();
    secondary_rotor_position_estimator_->reset_estimation();

    // Set the internal params pointer
    params_ = params;

    // Reset the PID controllers
    pid_d_current_.reset();
    pid_q_current_.reset();

    // Load the default id reference
    i_d_reference_ = params_->foc_params.i_d_reference_default;

    // reset the status
    status_.reset();

    // Set the state to stop
    state_ = BrushlessControlLoop::BrushlessControlLoopState::STOP;
}

BrushlessControlLoop::BrushlessControlLoopState BrushlessControlLoop::get_desired_state(
    float motor_speed, const BrushlessControlLoopState current_state) {
    BrushlessControlLoop::BrushlessControlLoopState desired_state = current_state;
    switch (current_state) {
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

    for (auto& error : errors) {
        error = false;
    }

    for (auto& warning : warnings) {
        warning = false;
    }
}

void BrushlessControlLoop::BrushlessControlLoopStatus::compute_base_status() {
    status = ControlLoopStatus::ControlLoopBaseStatus::OK;

    // If there's a warning, then set the status to warning
    for (auto warning : warnings) {
        if (warning != false) {
            status = ControlLoopStatus::ControlLoopBaseStatus::WARNING;
            break;
        }
    }

    // If there's an error, then set the status to error
    for (auto error : errors) {
        if (error != false) {
            status = ControlLoopStatus::ControlLoopBaseStatus::ERROR;
            break;
        }
    }
}

void BrushlessControlLoop::BrushlessControlLoopStatus::set_error(
    const BrushlessControlLoopStatus::BrushlessControlLoopError& error, bool value) {
    bool& error_ref = errors[static_cast<int>(error)];
    // This is done to avoid the cost of a branch
    if (error_ref != value) {
        error_ref = value;
        compute_base_status();
    }
}

void BrushlessControlLoop::BrushlessControlLoopStatus::set_warning(
    const BrushlessControlLoopStatus::BrushlessControlLoopWarning& warning, bool value) {
    bool& warning_ref = warnings[static_cast<int>(warning)];
    // This is done to avoid the cost of a branch
    if (warning_ref != value) {
        warning_ref = value;
        compute_base_status();
    }
}

ControlLoop::ControlLoopStatus BrushlessControlLoop::run_current_control(float i_d_reference, float i_q_reference) {
    if (params_->commutation_type != BrushlessControlLoopCommutationType::FOC) {
        // Set a warning in the status
        status_.set_error(BrushlessControlLoopStatus::BrushlessControlLoopError::CURRENT_CONTROL_NOT_SUPPORTED, true);
    } else {
        // Update the id reference
        i_d_reference_ = i_d_reference;
        // Now, run the FOC control loop with the speed divided by the speed to iq gain
        IGNORE(run(i_q_reference / params_->foc_params.speed_to_iq_gain));
    }

    return status_;
}

ControlLoop::ControlLoopStatus BrushlessControlLoop::run(float speed) {
    // Get the current time
    utime_t current_time_us = clock_.get_time_us();
    do {
        if (params_ == nullptr) {
            // Set an error in the status
            status_.set_error(BrushlessControlLoopStatus::BrushlessControlLoopError::PARAMS_NOT_SET, true);
            break;
        }

        // Clamp the speed
        math::clamp(speed, -1.0f, 1.0f);

        hwbridge::Bridge3Phase::phase_command_t phase_commands[3] = {0, false};

        // Get the current state and the desired state
        BrushlessControlLoop::BrushlessControlLoopState desired_state = get_desired_state(speed, state_);

        // If the desired state is different from the current state, then we need to transition
        if (desired_state != state_) {
            // Exit the current state
            exit_state(state_, desired_state);

            // Enter the desired state
            enter_state(state_, desired_state);
            state_ = desired_state;
        }

        // Update the rotor position estimator
        bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
        update_rotor_position_estimator(estimator_inputs, current_time_us, speed);

        // Run the state machine
        switch (state_) {
            case BrushlessControlLoop::BrushlessControlLoopState::STOP:
                break;

            case BrushlessControlLoop::BrushlessControlLoopState::RUN: {
                switch (params_->commutation_type) {
                    case BrushlessControlLoopCommutationType::FOC: {
                        run_foc(speed, current_time_us, last_run_time_, estimator_inputs.phase_current, phase_commands);
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
        app_hal_status_E status = this->bridge_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);

        // Set an error in the status
        status_.set_error(BrushlessControlLoopStatus::BrushlessControlLoopError::PHASE_COMMAND_FAILURE,
                          static_cast<bool>(status != app_hal_status_E::APP_HAL_OK));

    } while (false);
    last_run_time_ = current_time_us;

    // If the control loop has an error, then we should issue the phase commands to stop the motor
    if (status_.status == ControlLoopStatus::ControlLoopBaseStatus::ERROR) {
        // Set the duty cycles to 0
        hwbridge::Bridge3Phase::phase_command_t phase_commands[3] = {0, false};
        this->bridge_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);
    }

    return status_;
}

void BrushlessControlLoop::update_rotor_position_estimator(
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs& estimator_inputs, utime_t current_time_us, float speed) {
    do {
        estimator_inputs.time = current_time_us;

        bridge_.read_bemf(estimator_inputs.phase_voltage);
        bridge_.read_current(estimator_inputs.phase_current);

        // Get the commutation step
        estimator_inputs.current_commutation_step = Bldc6Step::determine_commutation_step_from_theta(rotor_position_);

        // Set the phase params
        estimator_inputs.phase_resistance = params_->foc_params.phase_resistance;
        estimator_inputs.phase_inductance = params_->foc_params.phase_inductance;

        // Set the PM flux linkage
        estimator_inputs.pm_flux_linkage = params_->foc_params.pm_flux_linkage;

        // Set the V alpha and V beta
        estimator_inputs.V_alpha = V_alpha_;
        estimator_inputs.V_beta = V_beta_;

        // Set the speed sign
        estimator_inputs.rotor_commanded_vel_sign = math::sign(speed);

        app_hal_status_E status = primary_rotor_position_estimator_.update(estimator_inputs);

        if (status != app_hal_status_E::APP_HAL_OK) {
            // Set an error in the status
            status_.set_warning(BrushlessControlLoopStatus::BrushlessControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE, true);
            break;
        }

        // TODO: For now we are just going to use the primary rotor position estimator
        // but the secondary rotor position estimator should be used if it is valid and the
        // primary rotor position estimator is not valid
        if (secondary_rotor_position_estimator_ != nullptr) {
            status = secondary_rotor_position_estimator_->update(estimator_inputs);
            if (status != app_hal_status_E::APP_HAL_OK) {
                // Set an error in the status
                status_.set_warning(BrushlessControlLoopStatus::BrushlessControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE,
                                    true);
                break;
            }
        }

        status_.set_warning(BrushlessControlLoopStatus::BrushlessControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE, false);
    } while (false);
}

void BrushlessControlLoop::exit_state(const BrushlessControlLoopState& current_state,
                                      const BrushlessControlLoopState& desired_state) {
    IGNORE(current_state);
    IGNORE(desired_state);
}

void BrushlessControlLoop::enter_state(const BrushlessControlLoopState& current_state,
                                       const BrushlessControlLoopState& desired_state) {
    IGNORE(current_state);
    switch (desired_state) {
        case BrushlessControlLoop::BrushlessControlLoopState::RUN: {
            if (params_->commutation_type == BrushlessControlLoopCommutationType::FOC) {
                // reset the PID controllers
                pid_d_current_.reset();
                pid_q_current_.reset();

                // Set the PI gains
                const float kp = params_->foc_params.current_control_bandwidth_rad_per_sec * params_->foc_params.phase_inductance;
                const float ki = params_->foc_params.phase_resistance / params_->foc_params.phase_inductance *
                                 kp;  // multiplied by kp to create a series PI controller

                pid_d_current_.set_kp(kp);
                pid_q_current_.set_kp(kp);
                if (params_->foc_params.disable_ki == false) {
                    pid_d_current_.set_ki(ki);
                    pid_q_current_.set_ki(ki);
                }

                // reset the rotor position estimator
                primary_rotor_position_estimator_.reset_estimation();
                if (secondary_rotor_position_estimator_ != nullptr) {
                    secondary_rotor_position_estimator_->reset_estimation();
                }
                // Set the desired rotor angle to the current rotor angle
                primary_rotor_position_estimator_.get_rotor_position(desired_rotor_angle_open_loop_);

                // Reset the debug vars
                foc_debug_vars_ = BldcFoc::FOCDebugVars();
            }
        } break;
        case BrushlessControlLoop::BrushlessControlLoopState::STOP:
        default:
            break;
    }
}

void BrushlessControlLoop::run_foc(float speed, utime_t current_time_us, utime_t last_run_time_us,
                                   hwbridge::Bridge3Phase::phase_current_t phase_currents,
                                   hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    do {
        const bool is_primary_estimator_valid = primary_rotor_position_estimator_.is_estimation_valid();
        bool is_secondary_estimator_valid = false;
        if (secondary_rotor_position_estimator_ != nullptr) {
            is_secondary_estimator_valid = secondary_rotor_position_estimator_->is_estimation_valid();
        }
        control_loop_type_ = get_desired_control_loop_type(is_primary_estimator_valid, is_secondary_estimator_valid);
        // Get the bus voltage
        float bus_voltage = 0.0f;
        app_hal_status_E status = bridge_.read_bus_voltage(bus_voltage);
        status_.set_error(BrushlessControlLoopStatus::BrushlessControlLoopError::BUS_VOLTAGE_READ_FAILURE,
                          static_cast<bool>(status != app_hal_status_E::APP_HAL_OK));
        if (status_.has_error(BrushlessControlLoopStatus::BrushlessControlLoopError::BUS_VOLTAGE_READ_FAILURE)) {
            break;
        }

        switch (control_loop_type_) {
            case BrushlessControlLoopType::OPEN_LOOP: {
                // increment the rotor position by the speed multiplied by the time since the last run
                desired_rotor_angle_open_loop_ += params_->open_loop_full_speed_theta_velocity * speed *
                                                  (float)(current_time_us - last_run_time_) /
                                                  basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;
                // Wrap the rotor position around 0 and 2pi
                math::wraparound(desired_rotor_angle_open_loop_, 0.0f, math::M_PI_FLOAT * 2.0f);

                rotor_position_ = desired_rotor_angle_open_loop_;

            } break;
            case BrushlessControlLoopType::CLOSED_LOOP: {
                // If the primary rotor position estimator is valid, then use it
                if (primary_rotor_position_estimator_.is_estimation_valid()) {
                    primary_rotor_position_estimator_.get_rotor_position(rotor_position_);
                    status_.set_warning(
                        BrushlessControlLoopStatus::BrushlessControlLoopWarning::PRIMARY_ROTOR_ESTIMATOR_NOT_VALID, false);
                }
                // Otherwise, if the secondary rotor position estimator is valid, then use it
                else if ((secondary_rotor_position_estimator_ != nullptr) &&
                         (secondary_rotor_position_estimator_->is_estimation_valid())) {
                    secondary_rotor_position_estimator_->get_rotor_position(rotor_position_);
                    status_.set_warning(
                        BrushlessControlLoopStatus::BrushlessControlLoopWarning::PRIMARY_ROTOR_ESTIMATOR_NOT_VALID, true);
                } else {
                    // Set an error in the status
                    status_.set_error(BrushlessControlLoopStatus::BrushlessControlLoopError::NO_VALID_ROTOR_POSITION_ESTIMATOR,
                                      true);
                    break;
                }
            } break;
            default:
                break;
        }
        // Do a Clarke transform
        math::clarke_transform_result_t clarke_transform =
            math::clarke_transform(phase_currents.u, phase_currents.v, phase_currents.w);

        i_alpha_ = clarke_transform.alpha;
        i_beta_ = clarke_transform.beta;

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
        // Limit the Vd and Vq by first calculating the modulus of the vector
        const float V_modulus = sqrtf(V_direct_ * V_direct_ + V_quadrature_ * V_quadrature_);
        const float max_Vmod = bus_voltage * 3.0f / 4.0f;
        // If the modulus is greater than the bus voltage, then we need to scale the voltage vector
        if (V_modulus > max_Vmod) {
            // Scale the voltage vector
            V_direct_ = V_direct_ * max_Vmod / V_modulus;
            V_quadrature_ = V_quadrature_ * max_Vmod / V_modulus;
        }

        // Determine the appropriate duty cycles for the inverter
        BldcFoc::FocDutyCycleResult result = BldcFoc::determine_inverter_duty_cycles_foc(
            rotor_position_, V_direct_, V_quadrature_, bus_voltage, params_->foc_params.pwm_control_type, phase_commands[0],
            phase_commands[1], phase_commands[2]);
        V_alpha_ = result.V_alpha;
        V_beta_ = result.V_beta;

        // Set the debug vars
        foc_debug_vars_.theta_e = rotor_position_;
        foc_debug_vars_.i_direct = i_direct_;
        foc_debug_vars_.i_quadrature = i_quadrature_;
        foc_debug_vars_.V_direct = V_direct_;
        foc_debug_vars_.V_quadrature = V_quadrature_;
        foc_debug_vars_.duty_cycle_u_h = result.duty_cycle_u_h;
        foc_debug_vars_.duty_cycle_v_h = result.duty_cycle_v_h;
        foc_debug_vars_.duty_cycle_w_h = result.duty_cycle_w_h;

    } while (false);
}

void BrushlessControlLoop::run_trap(float speed, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    // Get the rotor position
    primary_rotor_position_estimator_.get_rotor_position(rotor_position_);

    // Get the commutation step
    Bldc6Step::commutation_step_t current_commutation_step = Bldc6Step::determine_commutation_step_from_theta(rotor_position_);

    // Determine the duty cycles for the inverter
    Bldc6Step::determine_inverter_duty_cycles_trap(phase_commands, current_commutation_step, speed);
}

BrushlessControlLoop::BrushlessControlLoopType BrushlessControlLoop::get_desired_control_loop_type(
    bool is_primary_estimator_valid, bool is_secondary_estimator_valid) {
    BrushlessControlLoop::BrushlessControlLoopType desired_control_loop_type =
        BrushlessControlLoop::BrushlessControlLoopType::OPEN_LOOP;
    if (is_primary_estimator_valid || is_secondary_estimator_valid) {
        desired_control_loop_type = BrushlessControlLoop::BrushlessControlLoopType::CLOSED_LOOP;
    }
    return desired_control_loop_type;
}

}  // namespace control_loop
