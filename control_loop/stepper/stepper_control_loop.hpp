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
            BRIDGE_A_FAILURE,
            BRIDGE_B_FAILURE,
            TOTAL_ERROR_COUNT,
        };
        enum class StepperControlLoopWarning {
            NO_WARNING,
            TOTAL_WARNING_COUNT,
        };
        StepperControlLoopStatus() : ControlLoopStatus() {}

        static constexpr uint8_t kNumErrors = static_cast<uint8_t>(StepperControlLoopError::TOTAL_ERROR_COUNT);
        static constexpr uint8_t kNumWarnings = static_cast<uint8_t>(StepperControlLoopWarning::TOTAL_WARNING_COUNT);

        std::array<bool, kNumErrors> errors = {false};
        std::array<bool, kNumWarnings> warnings = {false};

        /**
         * @brief set the error
         * @param error The error to set
         * @param state The state to set the error to
         * @return void
         */
        void set_error(const StepperControlLoopError& error, const bool state);

        /**
         * @brief set the warning
         * @param warning The warning to set
         * @param state The state to set the warning to
         * @return void
         */
        void set_warning(const StepperControlLoopWarning& warning, const bool state);

        /**
         * @brief compute the base status returned by the class
         * @return True if the error is set
         */
        bool get_error(const StepperControlLoopError& error) const { return errors[static_cast<uint8_t>(error)]; }

        /**
         * @brief compute the base status returned by the class
         * @return True if the warning is set
         */
        bool get_warning(const StepperControlLoopWarning& warning) const { return warnings[static_cast<uint8_t>(warning)]; }

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

    ControlLoopStatus run(float speed) override;

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
    float current_setpoint_a = 0.0f, current_setpoint_b = 0.0f;

    // Create a status object
    StepperControlLoopStatus status_;
};

}  // namespace control_loop

#endif