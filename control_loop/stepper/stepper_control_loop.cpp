#include "stepper_control_loop.hpp"

#include "math.h"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {

void StepperControlLoop::init(StepperControlLoopParams* params) {
    params_ = params;

    // Set the previous time to the current time
    previous_time_ = clock_.get_time_us();

    // Reset the status
    status_.reset();
}

void StepperControlLoop::StepperControlLoopStatus::reset() {
    // Reset the status
    status = ControlLoopStatus::ControlLoopBaseStatus::OK;

    // Reset the error
    for (auto& error : errors) {
        error = false;
    }

    // Reset the warning
    for (auto& warning : warnings) {
        warning = false;
    }
}

void StepperControlLoop::StepperControlLoopStatus::set_error(const StepperControlLoopError& error, const bool state) {
    bool& error_ref = errors[static_cast<uint8_t>(error)];
    if (error_ref != state) {
        error_ref = state;
        compute_base_status();
    }
}

void StepperControlLoop::StepperControlLoopStatus::set_warning(const StepperControlLoopWarning& warning, const bool state) {
    bool& warning_ref = warnings[static_cast<uint8_t>(warning)];
    if (warning_ref != state) {
        warning_ref = state;
        compute_base_status();
    }
}

void StepperControlLoop::StepperControlLoopStatus::compute_base_status() {
    status = ControlLoopStatus::ControlLoopBaseStatus::OK;

    // If a warning is set, then set the status to WARNING
    for (auto& warning : warnings) {
        if (warning) {
            status = ControlLoopStatus::ControlLoopBaseStatus::WARNING;
            break;
        }
    }

    // If an error is set, then set the status to ERROR
    for (auto& error : errors) {
        if (error) {
            status = ControlLoopStatus::ControlLoopBaseStatus::ERROR;
            break;
        }
    }
}

// Run the stepper control loop
ControlLoop::ControlLoopStatus StepperControlLoop::run(float speed) {
    do {
        if (params_ == nullptr) {
            // If the params are not set, then return an error
            status_.set_error(StepperControlLoopStatus::StepperControlLoopError::PARAMS_NOT_SET, true);
            break;
        } else {
            // If the params are set, then return no error
            status_.set_error(StepperControlLoopStatus::StepperControlLoopError::PARAMS_NOT_SET, false);
        }
        // First, integrate the electrical angle based on the speed
        const float dt = (clock_.get_time_us() - previous_time_) / clock_.kMicrosecondsPerSecond;
        electrical_angle_ += speed * params_->max_speed * dt;
        // Wrap the electrical angle around 0 and 2pi
        math::wraparound(electrical_angle_, 0.0f, 2.0f * math::M_PI_FLOAT);

        // If the fabs speed is 0, then we are not moving, so set the current setpoints to the hold current
        const float current_setpoint = (fabs(speed) < math::ACCEPTABLE_FLOAT_ERROR) ? params_->i_hold : params_->i_run;

        // Determine the current setpoints
        auto current_setpoint_scalars = determine_current_setpoints(current_setpoint, electrical_angle_);

        // Run the current controllers of the 2 brushed control loops
        ControlLoopStatus status_a = bridge_a_.run_constant_current(current_setpoint_scalars.first);
        ControlLoopStatus status_b = bridge_b_.run_constant_current(current_setpoint_scalars.second);

        status_.set_error(StepperControlLoopStatus::StepperControlLoopError::BRIDGE_A_FAILURE,
                          static_cast<bool>(status_a != ControlLoopStatus::ControlLoopBaseStatus::OK));
        status_.set_error(StepperControlLoopStatus::StepperControlLoopError::BRIDGE_B_FAILURE,
                          static_cast<bool>(status_b != ControlLoopStatus::ControlLoopBaseStatus::OK));

    } while (false);

    return status_;
}

std::pair<float, float> StepperControlLoop::determine_current_setpoints(float desired_current, float electrical_angle) {
    // Determine the A and B current scalars based on the electrical angle
    // The A current scalar is the cosine of the electrical angle
    // The B current scalar is the sine of the electrical angle
    float a_current_scalar = cos(electrical_angle);
    float b_current_scalar = sin(electrical_angle);
    return std::make_pair(a_current_scalar * desired_current, b_current_scalar * desired_current);
}

}  // namespace control_loop