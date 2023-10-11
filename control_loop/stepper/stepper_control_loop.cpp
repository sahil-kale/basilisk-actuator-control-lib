#include "stepper_control_loop.hpp"

#include "math.h"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {

void StepperControlLoop::init(StepperControlLoopParams* params) {
    params_ = params;

    // Initialize the current controllers
    current_controller_a_.set_kp(params_->kp);
    current_controller_a_.set_ki(params_->ki);
    current_controller_a_.set_kd(params_->kd);

    current_controller_b_.set_kp(params_->kp);
    current_controller_b_.set_ki(params_->ki);
    current_controller_b_.set_kd(params_->kd);

    // Set the current setpoints to 0
    current_setpoint_a = 0;
    current_setpoint_b = 0;

    // Set the duty cycles to 0
    duty_cycle_a_ = 0;
    duty_cycle_b_ = 0;

    // Set the previous time to the current time
    previous_time_ = clock_.get_time_us();
}

// Run the stepper control loop
void StepperControlLoop::run(float speed) {
    do {
        if (params_ == nullptr) {
            // If the params are not set, return
            break;
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

        // Calculate the current error for the A and B motors
        // First, get the current values from the HBridge objects
        float current_a = 0.0f;
        bridge_a_.get_current(current_a);
        float current_b = 0.0f;
        bridge_b_.get_current(current_b);

        // If the duty cycle of the associated bridge is < 0, then the current is negative
        if (duty_cycle_a_ < 0) {
            current_a *= -1;
        }

        if (duty_cycle_b_ < 0) {
            current_b *= -1;
        }

        // Calculate the PID outputs
        float output_a = current_controller_a_.calculate(current_setpoint_scalars.first, current_a);
        float output_b = current_controller_b_.calculate(current_setpoint_scalars.second, current_b);

        // Calculate the duty cycles
        duty_cycle_a_ = output_a / params_->i_run;
        duty_cycle_b_ = output_b / params_->i_run;

    } while (false);
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