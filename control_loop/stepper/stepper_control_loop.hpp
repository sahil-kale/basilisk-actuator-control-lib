#ifndef STEPPER_CONTROL_LOOP_HPP
#define STEPPER_CONTROL_LOOP_HPP
#include <utility>

#include "brushed_control_loop.hpp"
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

        float max_speed;  // The maximum electrical speed of the motor (radians / second)
        // NOTE: the maximum speed is not the same as the maximum mechanical speed, which is the maximum electrical speed divided
        // by the number of pole pairs
    };

    class StepperControlLoopStatus : public ControlLoopStatus {
       public:
        enum class StepperControlLoopError {
            NO_ERROR,
            MOTOR_SPEED_TOO_HIGH,
            PARAMS_NOT_SET,
        };
        enum class StepperControlLoopWarning {
            NO_WARNING,
        };
        StepperControlLoopStatus() : ControlLoopStatus() {}
        StepperControlLoopWarning warning = StepperControlLoopWarning::NO_WARNING;
        StepperControlLoopError error = StepperControlLoopError::NO_ERROR;

        /**
         * @brief reset the status
         * @return void
         */
        void reset();

        /**
         * @brief compute the base status returned by the class
         * @return void
         */
        void compute_base_status();
    };

    // Provide a const getter for the status
    const StepperControlLoopStatus& get_status() const { return status_; }

    /**
     * @brief Construct a new StepperControlLoop object
     * @param bridge_a The HBridge object for the A motor
     * @param bridge_b The HBridge object for the B motor
     * @param clock The HAL_CLOCK object
     * @return void
     */
    StepperControlLoop(BrushedControlLoop& bridge_a, BrushedControlLoop& bridge_b, basilisk_hal::HAL_CLOCK& clock)
        : bridge_a_(bridge_a), bridge_b_(bridge_b), clock_(clock) {}

    void init(StepperControlLoopParams* params);

    ControlLoopStatus run(float speed);

   protected:
    /**
     * @brief determine the current setpoint scalars based on the electrical angle
     * @param electrical_angle The electrical angle of the motor (radians)
     * @return std::pair<float, float> The A and B current scalars
     */
    std::pair<float, float> determine_current_setpoints(float desired_current, float electrical_angle);

    BrushedControlLoop& bridge_a_;
    BrushedControlLoop& bridge_b_;
    basilisk_hal::HAL_CLOCK& clock_;
    float electrical_angle_ = 0;  // The electrical angle of the motor (radians)
    uint32_t previous_time_ = 0;
    StepperControlLoopParams* params_ = nullptr;

    // Create 2 variables to store the current setpoints for the A and B motors
    float current_setpoint_a, current_setpoint_b = 0;

    // Create a status object
    StepperControlLoopStatus status_;
};

}  // namespace control_loop

#endif