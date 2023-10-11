#ifndef STEPPER_CONTROL_LOOP_HPP
#define STEPPER_CONTROL_LOOP_HPP
#include <utility>

#include "bridge_hbridge.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "pid.hpp"

namespace control_loop {

class StepperControlLoop : public ControlLoop {
   public:
    class StepperControlLoopParams {
       public:
        float i_hold;  // The current to hold the motor at when not moving at the specific electrical angle
        float i_run;   // The current to run the motor at when moving at the specific electrical angle

        float kp;  // The proportional gain for the current control loop
        float ki;  // The integral gain for the current control loop
        float kd;  // The derivative gain for the current control loop

        float max_speed;  // The maximum electrical speed of the motor (radians / second)
        // NOTE: the maximum speed is not the same as the maximum mechanical speed, which is the maximum electrical speed divided
        // by the number of pole pairs
    };

    /**
     * @brief Construct a new StepperControlLoop object
     * @param bridge_a The HBridge object for the A motor
     * @param bridge_b The HBridge object for the B motor
     * @param clock The HAL_CLOCK object
     * @return void
     */
    StepperControlLoop(hwbridge::HBridge& bridge_a, hwbridge::HBridge& bridge_b, basilisk_hal::HAL_CLOCK& clock)
        : bridge_a_(bridge_a),
          bridge_b_(bridge_b),
          clock_(clock),
          current_controller_a_(0, 0, 0, 0, 0, 0),
          current_controller_b_(0, 0, 0, 0, 0, 0) {}

    void init(StepperControlLoopParams* params);

    void run(float speed);

   protected:
    /**
     * @brief determine the current setpoint scalars based on the electrical angle
     * @param electrical_angle The electrical angle of the motor (radians)
     * @return std::pair<float, float> The A and B current scalars
     */
    std::pair<float, float> determine_current_setpoints(float desired_current, float electrical_angle);

    hwbridge::HBridge& bridge_a_;
    hwbridge::HBridge& bridge_b_;
    basilisk_hal::HAL_CLOCK& clock_;
    float electrical_angle_ = 0;  // The electrical angle of the motor (radians)
    uint32_t previous_time_ = 0;
    StepperControlLoopParams* params_ = nullptr;

    // Create 2 PID objects for the A and B motors
    pid::PID<float> current_controller_a_;
    pid::PID<float> current_controller_b_;

    // Create 2 variables to store the current setpoints for the A and B motors
    float current_setpoint_a, current_setpoint_b = 0;
    // Further, store the last commanded duty cycles
    float duty_cycle_a_, duty_cycle_b_ = 0;
};

}  // namespace control_loop

#endif