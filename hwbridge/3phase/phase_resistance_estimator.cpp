#include "phase_resistance_estimator.hpp"

#include <cmath>

#include "math_util.hpp"

namespace hwbridge {
void PhaseResistanceEstimatorController::init(const Params& params) {
    params_ = params;
    state_ = State::NOT_STARTED;
    brake_start_time_ = 0;
    measurement_start_time_ = 0;
    commanded_voltage_ = 0.0f;
    current_controller_.set_kp(params_.current_kp);
    current_controller_.set_ki(params_.current_ki);
    current_controller_.set_kd(0.0f);
    current_controller_.reset();
}

PhaseResistanceEstimatorController::Result PhaseResistanceEstimatorController::run_phase_resistance_estimator(Input input) {
    Result result;
    // Get the current time
    const utime_t current_time = clock_.get_time_us();
    state_ = desired_state(current_time, input);
    const hwbridge::Bridge3Phase::phase_command_t brake_command = {0.0f, true};
    const hwbridge::Bridge3Phase::phase_command_t high_z_command = {0.0f, false};
    switch (state_) {
        case State::BRAKE_ROTOR: {
            if (brake_start_time_ == 0) {
                brake_start_time_ = current_time;
            }
            result.phase_commands[0] = brake_command;
            result.phase_commands[1] = brake_command;
            result.phase_commands[2] = brake_command;
            break;
        }
        case State::MEASUREMENT_IN_PROGRESS: {
            if (measurement_start_time_ == 0) {
                measurement_start_time_ = current_time;
            }

            // Run the current controller
            const float dv = current_controller_.calculate(input.phase_currents.u, params_.target_current);
            commanded_voltage_ += dv;

            float duty_cycle = commanded_voltage_ / (2 * input.bus_voltage) + 0.5f;
            // Clamp the duty cycle to be between 0 and 1
            math::clamp(duty_cycle, 0.0f, 1.0f);

            // We ground the U phase and short the V and W phases to the output of the current controller
            result.phase_commands[0] = brake_command;
            result.phase_commands[1] = {duty_cycle, true};
            result.phase_commands[2] = {duty_cycle, true};
            break;
        } break;
        case State::ESTIMATE_COMPLETE: {
            result.is_phase_resistance_valid = true;
            // Calculate the phase resistance
            // Note: the R_s is the phase inductance in a 3-phase system. When we measure the current in the singular phase that
            // is grounded, because we short 2 of the phases to Vbus, we end up with the resistance and inductance measurement of
            // the combined system, which is 3/2 of the phase inductance and resistance. So we need to divide by 3/2 to get the
            // phase resistance.
            result.phase_resistance = (2.0f / 3.0f) * commanded_voltage_ / params_.target_current;
        } break;
        case State::ERROR:
        case State::NOT_STARTED:
        default:
            result.phase_commands[0] = high_z_command;
            result.phase_commands[1] = high_z_command;
            result.phase_commands[2] = high_z_command;
            result.is_phase_resistance_valid = false;
            break;
    }

    result.state = state_;
    return result;
}

PhaseResistanceEstimatorController::State PhaseResistanceEstimatorController::desired_state(utime_t current_time,
                                                                                            Input input) const {
    State ret = state_;
    switch (state_) {
        case State::NOT_STARTED: {
            if (params_.measurement_duration == 0) {
                ret = State::ERROR;
            } else if (params_.brake_duration > 0) {
                ret = State::BRAKE_ROTOR;
            } else {
                ret = State::MEASUREMENT_IN_PROGRESS;
            }
            break;
        }
        case State::BRAKE_ROTOR: {
            const utime_t brake_end_time = brake_start_time_ + params_.brake_duration;
            if (current_time >= brake_end_time) {
                ret = State::MEASUREMENT_IN_PROGRESS;
            }
            break;
        } break;
        case State::MEASUREMENT_IN_PROGRESS: {
            const utime_t measurement_end_time = measurement_start_time_ + params_.measurement_duration;
            if (current_time >= measurement_end_time) {
                const float current_diff = fabs(params_.target_current - input.phase_currents.u);
                const bool is_current_within_tolerance = (current_diff <= params_.current_tolerance);
                if (is_current_within_tolerance) {
                    ret = State::ESTIMATE_COMPLETE;
                } else {
                    ret = State::ERROR;
                }
            }
        } break;
        case State::ESTIMATE_COMPLETE:
        case State::ERROR:
        default:
            // Persist in the current state
            break;
    }

    return ret;
}

}  // namespace hwbridge