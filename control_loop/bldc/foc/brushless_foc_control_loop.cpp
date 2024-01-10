#include "brushless_foc_control_loop.hpp"

#include "6step_util.hpp"
#include "bridge_3phase.hpp"
#include "foc_util.hpp"
#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {
using namespace BldcFoc;

// Define the init function
void BrushlessFOCControlLoop::init(Params* params) {
    do {
        // Initialize the rotor position estimator
        if (reset_position_estimators(rotor_position_estimators_, num_rotor_position_estimators_) == false) {
            break;  // Don't initialize if there was an issue resetting the rotor position estimators
        }

        // reset the status
        status_.reset();

        // Set the state to stop
        state_ = State::STOP;

        // Set the internal params pointer
        params_ = params;
    } while (false);
}

BrushlessFOCControlLoop::State BrushlessFOCControlLoop::get_desired_state(float i_q_reference, const State current_state,
                                                                          const Params& params,
                                                                          const BrushlessFOCControlLoopStatus& status) {
    State desired_state = current_state;
    const bool i_q_reference_is_zero = math::float_equals(i_q_reference, 0.0f);
    switch (current_state) {
        case State::STOP: {
            const bool has_error = (status == ControlLoop::ControlLoopBaseStatus::ERROR);
            if ((i_q_reference_is_zero == false) && (has_error == false)) {
                // We want to start the motor
                if (params.foc_params.phase_resistance_valid && params.foc_params.phase_inductance_valid) {
                    desired_state = State::RUN;
                } else {
                    desired_state = State::CALIBRATION;
                }
            }
        } break;
        case State::RUN: {
            if (i_q_reference_is_zero) {
                desired_state = State::STOP;
            }
        } break;
        case State::CALIBRATION: {
            const bool phase_resistance_failed =
                status.has_error(BrushlessFOCControlLoopError::PHASE_RESISTANCE_ESTIMATOR_FAILURE);
            const bool phase_inductance_failed =
                status.has_error(BrushlessFOCControlLoopError::PHASE_INDUCTANCE_ESTIMATOR_FAILURE);
            const bool phase_estimation_failed = phase_resistance_failed || phase_inductance_failed;
            if (params.foc_params.phase_resistance_valid && params.foc_params.phase_inductance_valid) {
                if (i_q_reference_is_zero) {
                    desired_state = State::STOP;
                } else {
                    desired_state = State::RUN;
                }
            } else if (phase_estimation_failed) {
                desired_state = State::STOP;
            }
        }
        default:
            // Unknown state
            break;
    }

    return desired_state;
}

ControlLoop::ControlLoopBaseStatus BrushlessFOCControlLoop::run(float speed) {
    // Clamp the speed
    math::clamp(speed, -this->MAX_MOTOR_SPEED, this->MAX_MOTOR_SPEED);
    // Now, run the FOC control loop with the speed multiplied by the speed to iq gain
    return run_current_control(params_->foc_params.i_d_reference_default, speed * params_->foc_params.speed_to_iq_gain);
}

ControlLoop::ControlLoopBaseStatus BrushlessFOCControlLoop::run_current_control(float i_d_reference, float i_q_reference) {
    // Get the current time
    utime_t current_time_us = clock_.get_time_us();
    hwbridge::Bridge3Phase::phase_command_t phase_commands[3];
    do {
        const bool params_valid = (params_ != nullptr);
        status_.set_error(BrushlessFOCControlLoopError::PARAMS_NOT_SET, (params_valid == false));
        // if the params not set error is set, then we should break
        if (status_.has_error(BrushlessFOCControlLoopError::PARAMS_NOT_SET)) {
            break;
        }

        math::direct_quad_t i_direct_quad_ref;
        i_direct_quad_ref.direct = i_d_reference;
        i_direct_quad_ref.quadrature = i_q_reference;

        // Update the FOC inputs
        FOCController::FOCInputs foc_inputs =
            update_foc_inputs(current_time_us, last_run_time_, rotor_position_estimators_, num_rotor_position_estimators_,
                              bridge_, status_, foc_frame_vars_.duty_cycle_result.V_alpha_beta, params_, i_direct_quad_ref);

        // Get the current state and the desired state
        State desired_state = get_desired_state(i_q_reference, state_, *params_, status_);

        // If the desired state is different from the current state, then we need to transition
        if (desired_state != state_) {
            // Exit the current state
            exit_state(state_, desired_state);

            // Enter the desired state
            enter_state(state_, desired_state);
            state_ = desired_state;
        }
        // Run the state machine
        switch (state_) {
            case State::STOP:
                break;

            case State::RUN: {
                if (status_ == ControlLoop::ControlLoopBaseStatus::ERROR) {
                    break;
                }
                foc_frame_vars_ = foc_controller_.run_foc(foc_inputs, phase_commands);
            } break;
            case State::CALIBRATION: {
                run_calibration(foc_inputs, phase_commands);
            } break;
            default:
                break;
        }

    } while (false);
    last_run_time_ = current_time_us;

    // If the control loop has an error, then we should issue the phase commands to stop the motor
    if (status_.status == ControlLoop::ControlLoopBaseStatus::ERROR) {
        // Set the duty cycles to 0
        hwbridge::Bridge3Phase::phase_command_t zero_duty_cycle;
        phase_commands[0] = zero_duty_cycle;
        phase_commands[1] = zero_duty_cycle;
        phase_commands[2] = zero_duty_cycle;
    }

    // Set the duty cycles
    app_hal_status_E hal_status = this->bridge_.set_phase(phase_commands[0], phase_commands[1], phase_commands[2]);

    // Set an error in the status
    status_.set_error(BrushlessFOCControlLoopError::PHASE_COMMAND_FAILURE,
                      static_cast<bool>(hal_status != app_hal_status_E::APP_HAL_OK));

    return status_.status;
}

void BrushlessFOCControlLoop::run_calibration(FOCController::FOCInputs inputs,
                                              hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    using namespace hwbridge;
    // Turn the FOC alpha/beta inputs into 3 phase current inputs
    const math::abc_t i_abc = math::inverse_clarke_transform(inputs.i_alpha_beta.alpha, inputs.i_alpha_beta.beta);
    const Bridge3Phase::phase_current_t phase_currents = {i_abc.a, i_abc.b, i_abc.c};

    if (params_->foc_params.phase_inductance_valid == false) {
        // Run the phase inductance estimator
        PhaseInductanceEstimatorController::Input input;
        input.phase_currents = phase_currents;
        input.bus_voltage = inputs.bus_voltage;
        PhaseInductanceEstimatorController::Result result = phase_inductance_estimator_.run_phase_inductance_estimator(input);
        params_->foc_params.phase_inductance = result.phase_inductance;
        params_->foc_params.phase_inductance_valid = result.is_phase_inductance_valid;

        phase_commands[0] = result.phase_commands[0];
        phase_commands[1] = result.phase_commands[1];
        phase_commands[2] = result.phase_commands[2];

        status_.set_error(BrushlessFOCControlLoopError::PHASE_INDUCTANCE_ESTIMATOR_FAILURE,
                          result.state == PhaseInductanceEstimatorController::State::ERROR);
    } else if (params_->foc_params.phase_resistance_valid == false) {
        PhaseResistanceEstimatorController::Input input;
        // Turn the FOC inputs into
        input.phase_currents = phase_currents;
        input.bus_voltage = inputs.bus_voltage;
        // Run the phase resistance estimator
        PhaseResistanceEstimatorController::Result result = phase_resistance_estimator_.run_phase_resistance_estimator(input);

        // Set the phase resistance
        params_->foc_params.phase_resistance = result.phase_resistance;
        params_->foc_params.phase_resistance_valid = result.is_phase_resistance_valid;

        phase_commands[0] = result.phase_commands[0];
        phase_commands[1] = result.phase_commands[1];
        phase_commands[2] = result.phase_commands[2];

        status_.set_error(BrushlessFOCControlLoopError::PHASE_RESISTANCE_ESTIMATOR_FAILURE,
                          result.state == PhaseResistanceEstimatorController::State::ERROR);
    }
}

void BrushlessFOCControlLoop::update_rotor_position_estimator(
    bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs& estimator_inputs, utime_t current_time_us,
    hwbridge::Bridge3Phase::phase_current_t phase_currents, const Params* params, math::alpha_beta_t V_alpha_beta,
    BrushlessFOCControlLoopStatus& status, float& theta,
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[], size_t num_rotor_position_estimators) {
    do {
        estimator_inputs.time = current_time_us;
        estimator_inputs.phase_current = phase_currents;

        // Set the phase params
        estimator_inputs.phase_resistance = params->foc_params.phase_resistance;
        estimator_inputs.phase_inductance = params->foc_params.phase_inductance;

        // Set the PM flux linkage
        estimator_inputs.pm_flux_linkage = params->foc_params.pm_flux_linkage;

        // Set the V alpha and V beta
        estimator_inputs.V_alpha_beta = V_alpha_beta;

        bool any_estimator_valid = false;
        bool any_estimator_failed_to_update = false;

        // Iterate through the rotor position estimators and update them
        for (size_t i = 0; i < num_rotor_position_estimators; i++) {
            if (rotor_position_estimators[i] == nullptr) {
                continue;
            }
            bldc_rotor_estimator::ElectricalRotorPosEstimator& estimator = *rotor_position_estimators[i];

            app_hal_status_E hal_status = estimator.update(estimator_inputs);
            if (hal_status == app_hal_status_E::APP_HAL_OK) {
                float tmp_theta = 0.0f;
                hal_status = estimator.get_rotor_position(tmp_theta);
                const bool is_valid = estimator.is_estimation_valid();
                const bool any_estimator_valid_rising_edge = (is_valid == true) && (any_estimator_valid == false);
                if (any_estimator_valid_rising_edge && (hal_status == app_hal_status_E::APP_HAL_OK)) {
                    theta = tmp_theta;
                    any_estimator_valid = true;
                }
            } else {
                any_estimator_failed_to_update = true;
            }
        }

        status.set_warning(BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR, any_estimator_valid == false);
        status.set_warning(BrushlessFOCControlLoopWarning::ROTOR_ESTIMATOR_UPDATE_FAILURE,
                           any_estimator_failed_to_update == true);

    } while (false);
}

void BrushlessFOCControlLoop::exit_state(const State& current_state, const State& desired_state) {
    IGNORE(current_state);
    IGNORE(desired_state);
}

void BrushlessFOCControlLoop::enter_state(const State& current_state, const State& desired_state) {
    IGNORE(current_state);
    switch (desired_state) {
        case State::RUN: {
            // Set the PI gains
            const float kp = params_->foc_params.current_control_bandwidth_rad_per_sec * params_->foc_params.phase_inductance;
            float ki = params_->foc_params.phase_resistance / params_->foc_params.phase_inductance *
                       kp;  // multiplied by kp to create a series PI controller

            if (params_->foc_params.disable_ki == false) {
                ki = 0.0f;
            }

            foc_controller_.init(kp, ki);

            // reset the rotor position estimator
            const bool position_estimators_reset_succesfully =
                reset_position_estimators(rotor_position_estimators_, num_rotor_position_estimators_);
            // TODO: Add a warning if the position estimators failed to reset
            IGNORE(position_estimators_reset_succesfully);

            // Reset the internal state
            foc_frame_vars_ = FOCController::FOCFrameVars();
        } break;
        case State::CALIBRATION: {
            phase_resistance_estimator_.init(params_->phase_resistance_estimator_params);
            phase_inductance_estimator_.init(params_->phase_inductance_estimator_params);
        } break;
        case State::STOP:
        default:
            break;
    }
}

bool BrushlessFOCControlLoop::reset_position_estimators(
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[], size_t num_rotor_position_estimators) {
    bool ret = false;
    do {
        // Initialize the rotor position estimator
        if (num_rotor_position_estimators == 0) {
            break;  // Don't initialize if there are no rotor position estimators
        }

        // Iterate through the rotor position estimators and reset them
        bool any_nullptr = false;
        for (uint8_t i = 0; i < num_rotor_position_estimators; i++) {
            if (rotor_position_estimators[i] == nullptr) {
                any_nullptr = true;
                break;
            }
            bldc_rotor_estimator::ElectricalRotorPosEstimator& estimator = *rotor_position_estimators[i];
            estimator.reset_estimation();
        }

        // If any of the rotor position estimators are null, then return
        if (any_nullptr) {
            break;
        }

        ret = true;

    } while (false);
    return ret;
}

FOCController::FOCInputs BrushlessFOCControlLoop::update_foc_inputs(
    utime_t current_time_us, utime_t last_run_time_us,
    bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[], size_t num_rotor_position_estimators,
    hwbridge::Bridge3Phase& bridge, BrushlessFOCControlLoopStatus& status, math::alpha_beta_t V_alpha_beta, const Params* params,
    math::direct_quad_t i_direct_quad_ref) {
    FOCController::FOCInputs foc_inputs;
    do {
        foc_inputs.timestamp = current_time_us;
        foc_inputs.dt = static_cast<float>(current_time_us - last_run_time_us) / 1e6f;
        hwbridge::Bridge3Phase::phase_current_t phase_currents;
        app_hal_status_E hal_status = bridge.read_phase_current(phase_currents);

        // Set an error in the status
        status.set_error(BrushlessFOCControlLoopError::PHASE_CURRENT_READ_FAILURE,
                         static_cast<bool>(hal_status != app_hal_status_E::APP_HAL_OK));

        foc_inputs.current_measurements_valid =
            status.has_error(BrushlessFOCControlLoopError::PHASE_CURRENT_READ_FAILURE) == false;

        // Read the bus voltage
        hal_status = bridge.read_bus_voltage(foc_inputs.bus_voltage);
        status.set_error(BrushlessFOCControlLoopError::BUS_VOLTAGE_READ_FAILURE,
                         static_cast<bool>(hal_status != app_hal_status_E::APP_HAL_OK));

        foc_inputs.bus_voltage_valid = status.has_error(BrushlessFOCControlLoopError::BUS_VOLTAGE_READ_FAILURE) == false;

        // Translate to A-B frame
        foc_inputs.i_alpha_beta = math::clarke_transform(phase_currents.u, phase_currents.v, phase_currents.w);

        // Update the rotor position estimator
        bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs estimator_inputs;
        update_rotor_position_estimator(estimator_inputs, current_time_us, phase_currents, params, V_alpha_beta, status,
                                        foc_inputs.theta_e, rotor_position_estimators, num_rotor_position_estimators);

        foc_inputs.rotor_position_valid =
            status.has_warning(BrushlessFOCControlLoopWarning::NO_VALID_ROTOR_POSITION_ESTIMATOR) == false;

        // Translate the currents to D-Q frame
        foc_inputs.i_direct_quad =
            math::park_transform(foc_inputs.i_alpha_beta.alpha, foc_inputs.i_alpha_beta.beta, foc_inputs.theta_e);

        foc_inputs.i_direct_quad_ref = i_direct_quad_ref;

        // Set the open loop params
        foc_inputs.open_loop_theta_velocity = params->open_loop_theta_velocity;
        foc_inputs.open_loop_quadrature_voltage = params->open_loop_quadrature_voltage;

        // Set the PWM control type
        foc_inputs.pwm_control_type = params->foc_params.pwm_control_type;

    } while (false);

    return foc_inputs;
}

FOCController::FOCFrameVars BrushlessFOCControlLoop::get_foc_frame_computation() const { return foc_frame_vars_; }

}  // namespace control_loop
