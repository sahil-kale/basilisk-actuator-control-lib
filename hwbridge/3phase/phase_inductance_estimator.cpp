#include "phase_inductance_estimator.hpp"

#include "math_util.hpp"

namespace hwbridge {

void PhaseInductanceEstimatorController::init(Params params) {
    params_ = params;
    state_ = State::NOT_STARTED;
    brake_start_time_ = 0;
    measurement_start_time_ = 0;
}

PhaseInductanceEstimatorController::Result PhaseInductanceEstimatorController::run_phase_inductance_estimator(
    PhaseInductanceEstimatorController::Input input) {
    Result result;
    // Get the current time
    const utime_t current_time = clock_.get_time_us();
    state_ = get_desired_state(current_time, input);

    const hwbridge::Bridge3Phase::phase_command_t brake_command = {0.0f, true};
    const hwbridge::Bridge3Phase::phase_command_t high_z_command = {0.0f, false};
    switch (state_) {
        case State::ERROR:
        case State::NOT_STARTED: {
            result.phase_commands[0] = high_z_command;
            result.phase_commands[1] = high_z_command;
            result.phase_commands[2] = high_z_command;
            result.is_phase_inductance_valid = false;
            break;
        }
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
            const hwbridge::Bridge3Phase::phase_command_t square_wave_command = {1.0f, true};
            if (measurement_start_time_ == 0) {
                measurement_start_time_ = current_time;
            }
            result.phase_commands[0] = brake_command;
            result.phase_commands[1] = square_wave_command;
            result.phase_commands[2] = square_wave_command;
            break;
        }
        case State::ESTIMATE_COMPLETE: {
            // High Z the bridge
            result.phase_commands[0] = high_z_command;
            result.phase_commands[1] = high_z_command;
            result.phase_commands[2] = high_z_command;

            // Calculate the phase inductance
            // TODO: Perhaps take an average of the bus voltage during the measurement period? But it's likely super fast anyways
            const float bus_voltage = input.bus_voltage;
            const float current =
                input.phase_currents.u * -1.0f;  // Negative current as positive current is going into the stator neutral
            // V = 3/2*L_s * di/dt
            // L_s = 2/3 * V / di/dt
            // Note: the L_s is the phase inductance in a 3-phase system. When we measure the current in the singular phase that
            // is grounded, because we short 2 of the phases to Vbus, we end up with the resistance and inductance measurement of
            // the combined system, which is 3/2 of the phase inductance and resistance. So we need to divide by 3/2 to get the
            // phase inductance.
            const float dt = clock_.get_dt_s(current_time, measurement_start_time_);
            const float di_dt = current / dt;
            result.phase_inductance = (2.0f / 3.0f) * bus_voltage / di_dt;
            result.is_phase_inductance_valid = true;
            break;
        }

        default:
            break;
    }

    result.state = state_;
    return result;
}

PhaseInductanceEstimatorController::State PhaseInductanceEstimatorController::get_desired_state(utime_t current_time,
                                                                                                Input input) const {
    State ret = state_;
    switch (state_) {
        case State::NOT_STARTED:
            if (params_.measurement_duration == 0) {
                ret = State::ERROR;
            } else if (params_.brake_duration > 0) {
                ret = State::BRAKE_ROTOR;
            } else {
                ret = State::MEASUREMENT_IN_PROGRESS;
            }
            break;
        case State::BRAKE_ROTOR: {
            const utime_t brake_end_time = brake_start_time_ + params_.brake_duration;
            if (current_time >= brake_end_time) {
                ret = State::MEASUREMENT_IN_PROGRESS;
            }
            break;
        }
        case State::MEASUREMENT_IN_PROGRESS: {
            const utime_t measurement_end_time = measurement_start_time_ + params_.measurement_duration;
            if (current_time >= measurement_end_time) {
                const bool is_current_nonzero = (math::float_equals(input.phase_currents.u, 0.0f) == false);
                const bool is_bus_voltage_nonzero = (math::float_equals(input.bus_voltage, 0.0f) == false);
                if (is_current_nonzero && is_bus_voltage_nonzero) {
                    ret = State::ESTIMATE_COMPLETE;
                } else {
                    ret = State::ERROR;
                }
            }
            break;
        }
        case State::ERROR:
        case State::ESTIMATE_COMPLETE:
        default:
            // Do not change the state
            ret = state_;
            break;
    }

    return ret;
}
}  // namespace hwbridge