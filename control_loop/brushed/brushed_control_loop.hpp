#ifndef BRUSHED_CONTROL_LOOP_HPP
#define BRUSHED_CONTROL_LOOP_HPP
#include <stdint.h>

#include <array>

#include "bridge_hbridge.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "pid.hpp"

namespace control_loop {

/**
 * @brief A control loop for a brushed motor
 * @attention This control loop is WIP and not fully implemented
 */
class BrushedControlLoop : public ControlLoop {
   public:
    /**
     * @brief Construct a new BrushedControlLoop object
     * @param bridge The h bridge to control
     * @param clock The clock to use for the control loop
     */
    BrushedControlLoop(hwbridge::HBridge& bridge, basilisk_hal::HAL_CLOCK& clock)
        : bridge_(bridge), clock_(clock), current_controller_{0, 0, 0, -1.0f, 1.0f, 0, clock_} {}

    /**
     * @brief The parameters for the current controller when running the control loop through current control mode
     */
    class BrushedControlLoopCurrentControllerParams {
       public:
        /**
         * @brief kp The proportional gain for the current control loop
         */
        float kp;
        /**
         * @brief ki The integral gain for the current control loop
         */
        float ki;
        /**
         * @brief kd The derivative gain for the current control loop
         */
        float kd;
    };

    /**
     * @brief The type of brake to apply to the motor when the control loop speed is 0
     */
    enum class BrushedBrakeType {
        /// Do not brake the motor when the control loop is not running, high-Z the h bridge
        COAST,
        /// Brake the motor by shorting the low side of the h bridge
        BRAKE_LOW_SIDE,
        /// Brake the motor by shorting the high side of the h bridge
        BRAKE_HIGH_SIDE,
    };

    /**
     * @brief The parameters for the control loop
     * @note The parameters must be set before the control loop can be run
     */
    class BrushedControlLoopParams {
       public:
        /**
         * @brief The type of brake to apply to the motor when the control loop speed is 0
         */
        BrushedBrakeType brake_mode;
        /**
         * @brief The deadtime (us) to apply to the h bridge to prevent shoot through
         */
        float deadtime_us;
        /**
         * @brief The current controller parameters to use when running the control loop through current control mode
         */
        BrushedControlLoopCurrentControllerParams current_controller_params;
    };

    /**
     * @brief The status of the control loop
     */
    class BrushedControlLoopStatus : public ControlLoopStatus {
       public:
        /**
         * @brief The errors of the brushed control loop
         */
        enum class BrushedControlLoopError {
            NO_ERROR,
            /// The parameters for the control loop have not been set (the param pointer is null)
            PARAMS_NOT_SET,
            /// Failed to get the current from the h bridge
            GET_CURRENT_FAILED,
            /// The h bridge has failed
            BRIDGE_FAILURE,
            TOTAL_ERROR_COUNT,
        };
        /**
         * @brief The warnings of the brushed control loop
         */
        enum class BrushedControlLoopWarning {
            NO_WARNING,
            TOTAL_WARNING_COUNT,
        };
        BrushedControlLoopStatus() : ControlLoopStatus() {}

        /**
         * @brief The number of errors of the control loop
         */
        static constexpr uint8_t kNumErrors = static_cast<uint8_t>(BrushedControlLoopError::TOTAL_ERROR_COUNT);
        /**
         * @brief The number of warnings in the control loop
         */
        static constexpr uint8_t kNumWarnings = static_cast<uint8_t>(BrushedControlLoopWarning::TOTAL_WARNING_COUNT);

        /**
         * @brief The errors of the control loop stored in an array
         * @note The order of the errors is the same as the order of the enum class
         */
        std::array<bool, kNumErrors> errors = {false};

        /**
         * @brief The warnings of the control loop stored in an array
         * @note The order of the warnings is the same as the order of the enum class
         */
        std::array<bool, kNumWarnings> warnings = {false};

        /**
         * @brief reset the status
         * @return void
         */
        void reset();

        /**
         * @brief Set the error state
         * @param error The error to set
         * @param state The state to set the error to
         * @return void
         */
        void set_error(const BrushedControlLoopError& error, const bool state);

        /**
         * @brief Set the warning state
         * @param warning The warning to set
         * @param state The state to set the warning to
         * @return void
         */
        void set_warning(const BrushedControlLoopWarning& warning, const bool state);

        /**
         * @brief Get the error state
         * @param error The error to get
         * @return The state of the error
         */
        bool get_error(const BrushedControlLoopError& error) const { return errors[static_cast<uint8_t>(error)]; }

        /**
         * @brief Get the warning state
         * @param warning The warning to get
         * @return The state of the warning
         */
        bool get_warning(const BrushedControlLoopWarning& warning) const { return warnings[static_cast<uint8_t>(warning)]; }

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
    const BrushedControlLoopStatus& get_status() const { return status_; }

    /**
     * @brief The state of the control loop
     */
    enum class BrushedControlLoopState {
        STOP,
        RUN,
        /// The control loop is paused for deadtime
        DEADTIME_PAUSE,
    };

    /**
     * @brief Initialize the control loop
     * @param params The parameters for the control loop
     * @return void
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    void init(BrushedControlLoopParams* params);

    /**
     * @brief Run the control loop
     * @param speed The speed to run the motor at
     * @return The status of the control loop
     */
    ControlLoopStatus run(float speed) override;

    /**
     * @brief Run the motor at a constant current
     * @param current The current to run the motor at
     * @return void
     */
    ControlLoopStatus run_constant_current(float current);

   protected:
    /*! \cond PRIVATE */
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

    // Define a status object
    BrushedControlLoopStatus status_;

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
    /*! \endcond */
};
}  // namespace control_loop

#endif