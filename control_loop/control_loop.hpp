#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

namespace control_loop {

/**
 * @brief An abstract class for a control loop
 */
class ControlLoop {
   public:
    /**
     * @brief The status of the control loop
     * @note This class is used to return the status of the control loop
     * @note This class should be extended by the control loop implementation to add additional status information if needed
     */
    class ControlLoopStatus {
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
    };

    /**
     * @brief Run the control loop
     * @param speed The speed to run the control loop at. -1.0f - 1.0f
     */
    virtual ControlLoopStatus run(float speed) = 0;

    /**
     * @brief Maximum motor speed permissable to be commanded
     */
    static constexpr float MAX_MOTOR_SPEED = 1.0f;
};

}  // namespace control_loop

#endif  // CONTROL_LOOP_HPP