#ifndef STEPPER_CONTROL_LOOP_HPP
#define STEPPER_CONTROL_LOOP_HPP

#if 0
#include <utility>

#include "brushed_control_loop.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "pid.hpp"

namespace control_loop {

/**
 * @brief A control loop for a stepper motor
 */
class StepperControlLoop : public ControlLoop {
   public:
    /**
     * @brief The parameters for the control loop
     */
    class StepperControlLoopParams {
       public:
        /**
         * @brief The current to hold the motor at when not moving at the specific electrical angle
         */
        float i_hold;
        /**
         * @brief The current to run the motor at when moving at the specific electrical angle
         */
        float i_run;
        /**
         * @brief steps per second (note: the electrical speed is then 2pi * max_speed)
         */
        float max_speed;
    };

    /**
     * @brief The status of the control loop
     */
    class StepperControlLoopStatus : public ControlLoopStatus {
       public:
        /**
         * @brief The error conditions for the stepper control loop
         */
        enum class StepperControlLoopError {
            NO_ERROR,
            /// The motor speed is too high and the control loop cannot keep up
            MOTOR_SPEED_TOO_HIGH,
            /// The control loop parameters have not been set
            PARAMS_NOT_SET,
            /// The A bridge has failed
            BRIDGE_A_FAILURE,
            /// The B bridge has failed
            BRIDGE_B_FAILURE,
            TOTAL_ERROR_COUNT,
        };
        /**
         * @brief The warning conditions for the stepper control loop
         */
        enum class StepperControlLoopWarning {
            NO_WARNING,
            TOTAL_WARNING_COUNT,
        };
        StepperControlLoopStatus() : ControlLoopStatus() {}

        /** Number of errors for the stepper control loop*/
        static constexpr uint8_t kNumErrors = static_cast<uint8_t>(StepperControlLoopError::TOTAL_ERROR_COUNT);
        /** Number of warnings for the stepper control loop */
        static constexpr uint8_t kNumWarnings = static_cast<uint8_t>(StepperControlLoopWarning::TOTAL_WARNING_COUNT);

        /**
         * @brief Array with the error states for the stepper control loop
         * @note The order of the errors is the same as the order of the enum class
         */
        std::array<bool, kNumErrors> errors = {false};
        /**
         * @brief Array with the warning states for the stepper control loop
         * @note The order of the warnings is the same as the order of the enum class
         */
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

    /**
     * @brief Get the status of the control loop
     * @return The status of the control loop
     */
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

    /**
     * @brief Initialize the control loop
     * @param params The parameters for the control loop
     * @return void
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    void init(StepperControlLoopParams* params);

    ControlLoopStatus run(float speed) override;

    /**
     * @brief get the number of steps the motor has taken
     * @return float The number of steps the motor has taken
     * @note This is the number of steps the motor has taken since the last call to init()
     */
    float get_steps() const { return steps_; }

    /**
     * @brief set the number of steps the motor has taken
     * @param step The number of steps the motor has taken
     * @note The steps_ variable also determines the electrical angle of the motor, as a result,
     *       this function may cause the motor to jump to a new electrical angle to align the rotor.
     *       It is expected that on most systems, the minor movement of the rotor will not be noticeable.
     */
    void set_steps(float step) { steps_ = step; }

   protected:
    /*! \cond PRIVATE */
    /**
     * @brief determine the current setpoint scalars based on the electrical angle
     * @param electrical_angle The electrical angle of the motor (radians)
     * @param desired_current The desired current to run the motor at
     * @return std::pair<float, float> The A and B current scalars
     */
    std::pair<float, float> determine_current_setpoints(float desired_current, float electrical_angle);

    BrushedControlLoop& bridge_a_;
    BrushedControlLoop& bridge_b_;
    basilisk_hal::HAL_CLOCK& clock_;

    float steps_ = 0;  // The number of steps the motor has taken

    uint32_t previous_time_ = 0;
    StepperControlLoopParams* params_ = nullptr;

    float current_setpoint_a = 0.0f, current_setpoint_b = 0.0f;

    /** The status of the control loop */
    StepperControlLoopStatus status_;
    /*! \endcond */
};

}  // namespace control_loop

#endif

#endif