#ifndef STEPPER_CONTROL_LOOP_HPP
#define STEPPER_CONTROL_LOOP_HPP
#include <utility>

#include "bridge_hbridge.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"

namespace control_loop {

class StepperControlLoop : public ControlLoop {
   public:
    class StepperControlLoopParams {
       public:
        float stepper_motor_current_to_pwm_duty_cycle_slope;
        bool stepper_motor_disable_current_pid;
        bool stepper_motor_simple_switcher_enabled;
    };

    // Define a constructor that takes 2 references to HBridge objects. One for the A motor and one for the B motor. Also, a clock
    // instance
    StepperControlLoop(hwbridge::HBridge& motor_a, hwbridge::HBridge& motor_b, basilisk_hal::HAL_CLOCK& clock)
        : motor_a_(motor_a), motor_b_(motor_b), clock_(clock) {}
    void run(float speed) override;

    // Define a function to determine the electrical angle of the motor
    /**
     * @brief Determine the electrical angle of the motor
     * @param time_delta The time delta since the last time this function was called in seconds
     * @param desired_speed The desired speed of the motor
     * @param previous_angle The previous angle of the motor
     *
     * @return The new angle of the motor in degrees (0 -> 360)
     */
    float determineElectricalAngle(float time_delta, float desired_speed, float previous_angle);

    /**
     * @brief Determine the electrical angle of the motor using 4 step commutation
     * @param time_delta The time delta since the last time this function was called in seconds
     * @param desired_speed The desired speed of the motor
     * @param previous_angle The previous angle of the motor
     */
    float determineElectricalAngleSimple(float time_delta, float desired_speed, float previous_angle);

    // Define a function to determine the AB and CD current magnitudes based on the electrical angle
    /**
     * @brief Determine the A and B current scalar (-1 -> 1) based on the electrical angle
     * @param electrical_angle The electrical angle of the motor
     * @return A pair of floats. The first is the A current scalar and the second is the B current scalar
     */
    std::pair<float, float> determineCurrentSetpointScalars(float electrical_angle);

   private:
    hwbridge::HBridge& motor_a_;
    hwbridge::HBridge& motor_b_;
    basilisk_hal::HAL_CLOCK& clock_;
    float previous_angle_ = 0.0f;
    uint32_t previous_time_ = 0;
    StepperControlLoopParams* params_ = nullptr;
};

}  // namespace control_loop

#endif