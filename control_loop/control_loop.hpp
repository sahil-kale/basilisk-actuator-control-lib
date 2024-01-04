#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

#include <cstdint>

namespace control_loop {

/**
 * @brief An abstract class for a control loop
 */
class ControlLoop {
   public:
    /**
     * @brief The status of the control loop in a simplified enum - this is used to return the status of the base control loop
     * to provide an easy to use API
     */
    enum class ControlLoopBaseStatus {
        /// The control loop is running normally
        OK,
        /// The control loop is running but there is a warning
        WARNING,
        /// The control loop is not running due to an error
        ERROR,
    };

    /**
     * @brief The status of the control loop
     * @param WarningEnum The enum class of the warnings
     * @param ErrorEnum The enum class of the errors
     * @note This class is used to return the status of the control loop
     */
    template <typename ErrorEnum, typename WarningEnum>
    class ControlLoopStatus {
       public:
        /**
         * @brief The status of the control loop
         */
        ControlLoopBaseStatus status = ControlLoopBaseStatus::OK;

        /**
         * @brief Operator to check if a status class is equal to a ControlLoopBaseStatus enum
         */
        bool operator==(ControlLoopBaseStatus status) const { return this->status == status; }

        /**
         * @brief Operator to check if a status class is not equal to a ControlLoopBaseStatus enum
         */
        bool operator!=(ControlLoopBaseStatus status) const { return this->status != status; }

        /**
         * @brief reset the status
         * @return void
         */
        void reset() {
            for (auto& error : errors) {
                error = false;
            }

            for (auto& warning : warnings) {
                warning = false;
            }

            compute_base_status();
        }

        /**
         * @brief Set the error
         * @param error The error to set
         * @param state The state of the error
         * @return void
         */
        void set_error(const ErrorEnum& error, bool state) {
            bool& error_ref = errors[static_cast<int>(error)];
            // This is done to avoid the cost of a branch
            if (error_ref != state) {
                error_ref = state;
                compute_base_status();
            }
        }

        /**
         * @brief Check if the error is set
         * @param error The error to check
         * @return True if the error is set
         */
        bool has_error(const ErrorEnum& error) const { return errors[static_cast<uint8_t>(error)]; }

        /**
         * @brief Set the warning
         * @param warning The warning to set
         * @param state The state of the warning
         * @return void
         */
        void set_warning(const WarningEnum& warning, bool state) {
            bool& warning_ref = warnings[static_cast<int>(warning)];
            // This is done to avoid the cost of a branch
            if (warning_ref != state) {
                warning_ref = state;
                compute_base_status();
            }
        }

        /**
         * @brief Check if the warning is set
         * @param warning The warning to check
         * @return True if the warning is set
         */
        bool has_warning(const WarningEnum& warning) const { return warnings[static_cast<uint8_t>(warning)]; }
        /**
         * @brief The number of errors of the control loop
         */
        static constexpr uint8_t kNumErrors = static_cast<uint8_t>(ErrorEnum::TOTAL_ERROR_COUNT);
        /**
         * @brief The number of warnings in the control loop
         */
        static constexpr uint8_t kNumWarnings = static_cast<uint8_t>(WarningEnum::TOTAL_WARNING_COUNT);

       private:
        /**
         * @brief The errors of the control loop stored in an array
         * @note The order of the errors is the same as the order of the enum class
         */
        bool errors[kNumErrors] = {false};
        bool warnings[kNumWarnings] = {false};

        /**
         * @brief compute the base status returned by the class
         * @return void
         */
        void compute_base_status() {
            status = ControlLoop::ControlLoopBaseStatus::OK;

            // If there's a warning, then set the status to warning
            for (auto warning : warnings) {
                if (warning != false) {
                    status = ControlLoop::ControlLoopBaseStatus::WARNING;
                    break;
                }
            }

            // If there's an error, then set the status to error
            for (auto error : errors) {
                if (error != false) {
                    status = ControlLoop::ControlLoopBaseStatus::ERROR;
                    break;
                }
            }
        }
    };

    /**
     * @brief Run the control loop
     * @param speed The speed to run the control loop at. -1.0f - 1.0f
     */
    virtual ControlLoopBaseStatus run(float speed) = 0;

    /**
     * @brief Maximum motor speed permissable to be commanded
     */
    static constexpr float MAX_MOTOR_SPEED = 1.0f;
};

}  // namespace control_loop

#endif  // CONTROL_LOOP_HPP