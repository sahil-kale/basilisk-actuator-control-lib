#include "brushed_control_loop.hpp"

#include "math.h"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {

void BrushedControlLoop::init(BrushedControlLoopParams* params) { params_ = params; }

void BrushedControlLoop::run(float speed) {
    // Get the current time
    utime_t current_time = clock_.get_time_us();
    // Get the desired state
    BrushedControlLoopState desired_state =
        get_desired_state(speed, last_speed_, state_, current_time, deadtime_start_time_, params_->deadtime_us);

    // If the desired state is different than the current state, then we need to transition to the desired state
    if (desired_state != state_) {
        // If the desired state is STOP, then we need to set the deadtime start time
        if (desired_state == BrushedControlLoopState::STOP) {
            deadtime_start_time_ = current_time;
        }
        // If the desired state is DEADTIME_PAUSE, then we need to set the deadtime start time
        else if (desired_state == BrushedControlLoopState::DEADTIME_PAUSE) {
            deadtime_start_time_ = current_time;
        }
        // Otherwise, we need to set the deadtime start time to 0
        else {
            deadtime_start_time_ = 0;
        }

        // Set the state to the desired state
        state_ = desired_state;
    }

    // Get the desired output
    hwbridge::HBridge::HBridgeInput bridge_input = run_state(speed, state_);

    // Run the bridge
    bridge_.run(bridge_input);

    // Update the last speed
    last_speed_ = speed;
}

BrushedControlLoop::BrushedControlLoopState BrushedControlLoop::get_desired_state(float desired_speed, float previous_speed,
                                                                                  BrushedControlLoopState current_state,
                                                                                  utime_t current_time,
                                                                                  utime_t deadtime_start_time,
                                                                                  utime_t deadtime_pause_time_us) {
    BrushedControlLoopState desired_state = current_state;

    switch (current_state) {
        case BrushedControlLoopState::STOP:
            if (desired_speed != 0) {
                // NOTE: we should check that we are in the stop state for at least DEADTIME_PAUSE_TIME_US before transitioning to
                // RUN
                desired_state = BrushedControlLoopState::RUN;
            }
            break;
        case BrushedControlLoopState::RUN: {
            if (fabs(desired_speed) < math::ACCEPTABLE_FLOAT_ERROR) {
                desired_state = BrushedControlLoopState::STOP;
            }
            // Otherwise, if the desired speed is different than the previous speed, then we need to pause the control loop for
            // deadtime
            else if ((desired_speed * previous_speed < 0) && (deadtime_pause_time_us > 0)) {
                // If the deadtime pause ticks is > 0, then we need to pause the control loop for deadtime
                desired_state = BrushedControlLoopState::DEADTIME_PAUSE;
            } else {
                // Do nothing, maintain the RUN state
                desired_state = BrushedControlLoopState::RUN;
            }
        } break;
        case BrushedControlLoopState::DEADTIME_PAUSE: {
            // If the speed is 0, then we can transition to the STOP state
            if (fabs(desired_speed) < math::ACCEPTABLE_FLOAT_ERROR) {
                desired_state = BrushedControlLoopState::STOP;
            }
            // Otherwise, check to see if the deadtime has expired
            else if ((current_time - deadtime_start_time) >= deadtime_pause_time_us) {
                desired_state = BrushedControlLoopState::RUN;
            } else {
                // Do nothing, maintain the DEADTIME_PAUSE state
                desired_state = BrushedControlLoopState::DEADTIME_PAUSE;
            }
        }
        default:
            break;
    }

    return desired_state;
}

hwbridge::HBridge::HBridgeInput BrushedControlLoop::run_state(float speed, const BrushedControlLoopState& state) {
    hwbridge::HBridge::HBridgeInput bridge_input = {0.0f, false, 0.0f, false};

    switch (state) {
        case BrushedControlLoopState::STOP: {
            switch (params_->brake_mode) {
                case BrushedBrakeType::COAST: {
                    bridge_input = {0.0f, false, 0.0f, false};
                }; break;
                case BrushedBrakeType::BRAKE_LOW_SIDE: {
                    bridge_input = {0.0f, true, 0.0f, true};
                }; break;
                case BrushedBrakeType::BRAKE_HIGH_SIDE: {
                    bridge_input = {1.0f, false, 1.0f, false};
                }; break;
                default:
                    // do nothing
                    break;
            };
        } break;
        case BrushedControlLoopState::RUN: {
            // If the speed is positive, then we need to drive the motor forward (A high, B low)
            // If the speed is negative, then we need to drive the motor backward (A low, B high)
            if (speed > 0) {
                bridge_input = {speed, false, 0.0f, true};
            } else {
                bridge_input = {0.0f, true, -speed, false};
            }
        } break;
        case BrushedControlLoopState::DEADTIME_PAUSE: {
            // Set the bridge to float
            bridge_input = {0.0f, false, 0.0f, false};
        } break;
        default:
            // Do nothing
            break;
    };

    return bridge_input;
}

}  // namespace control_loop