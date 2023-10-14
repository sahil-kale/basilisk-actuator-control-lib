#ifndef BRUSHED_CONTROL_LOOP_HPP
#define BRUSHED_CONTROL_LOOP_HPP
#include <stdint.h>

#include "bridge_hbridge.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "pid.hpp"

namespace control_loop {

// Define a class BrushedControlLoop that inherits from ControlLoop

class BrushedControlLoop : public ControlLoop {
   public:
    BrushedControlLoop(hwbridge::HBridge& bridge, basilisk_hal::HAL_CLOCK& clock)
        : bridge_(bridge), clock_(clock), current_controller_{0, 0, 0, 1.0f, -1.0f, 0, clock_} {}

    class BrushedControlLoopCurrentControllerParams {
       public:
        float kp;  // The proportional gain for the current control loop
        float ki;  // The integral gain for the current control loop
        float kd;  // The derivative gain for the current control loop
    };

    enum class BrushedBrakeType {
        COAST,
        BRAKE_LOW_SIDE,
        BRAKE_HIGH_SIDE,
    };

    class BrushedControlLoopParams {
       public:
        BrushedBrakeType brake_mode;  // Whether or not to brake the motor when the control loop is not running
        float deadtime_us;            // The deadtime to apply to the h bridge to prevet shoot through
        BrushedControlLoopCurrentControllerParams current_controller_params;
    };

    enum class BrushedControlLoopState {
        STOP,
        RUN,
        DEADTIME_PAUSE,
    };

    /**
     * @brief Initialize the control loop
     * @param params The parameters for the control loop
     * @return void
     */
    void init(BrushedControlLoopParams* params);

    void run(float speed) override;

    /**
     * @brief Run the motor at a constant current
     * @param current The current to run the motor at
     * @return void
     */
    void run_constant_current(float current);

   protected:
    hwbridge::HBridge& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    BrushedControlLoopParams* params_ = nullptr;

    BrushedControlLoopState state_ = BrushedControlLoopState::STOP;
    float last_speed_ = 0.0f;
    utime_t deadtime_start_time_ = 0;

    // Define a PID controller for the current control loop
    pid::PID<float> current_controller_;
    // Define a variable to store the duty cycle commanded by the current controller
    float current_controller_duty_cycle_ = 0.0f;

    /**
     * @brief Get the desired state of the control loop
     * @param desired_speed The desired speed of the motor
     * @param previous_speed The previous speed of the motor
     * @param current_state The current state of the control loop
     * @param current_time The current time
     * @param deadtime_start_time The time at which the deadtime started
     * @param deadtime_pause_time_us The number of ticks to pause the control loop for during deadtime
     * @return The desired state of the control loop
     */
    BrushedControlLoopState get_desired_state(float desired_speed, float previous_speed, BrushedControlLoopState current_state,
                                              utime_t current_time, utime_t deadtime_start_time, utime_t deadtime_pause_time_us);

    /**
     * @brief run and get the bridge input for a given speed and state
     * @param speed The speed to run the motor at
     * @param state The state of the control loop
     */
    hwbridge::HBridge::HBridgeInput run_state(float speed, const BrushedControlLoopState& state);
};
}  // namespace control_loop

#endif