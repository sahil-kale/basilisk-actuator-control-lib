#ifndef BRUSHLESS_6STEP_CONTROL_LOOP_HPP
#define BRUSHLESS_6STEP_CONTROL_LOOP_HPP

#include "6step_util.hpp"
#include "control_loop.hpp"
#include "rotor_estimator.hpp"

namespace control_loop {

/**
 * @brief A control loop for a brushless motor using 6 step commutation
 */
class Brushless6StepControlLoop : public ControlLoop {
   public:
    /**
     * @brief Construct a new Brushless 6 Step Control Loop object
     * @param motor The bridge to control
     * @param rotor_position_estimator The rotor position estimator to use
     * @note The rotor position estimator must be initialized before calling run
     */
    Brushless6StepControlLoop(hwbridge::Bridge3Phase& motor,
                              bldc_rotor_estimator::ElectricalRotorPosEstimator& rotor_position_estimator)
        : motor_(motor), rotor_position_estimator_(rotor_position_estimator) {}

    /**
     * @brief The warnings for the brushless 6 step control loop
     */
    enum class Brushless6StepWarning {
        /// @brief The phase voltage read has failed - if the estimator is sensorless, this means the rotor position cannot be
        /// deduced
        PHASE_VOLTAGE_READ_FAILURE,
        /// @brief Total number of warnings
        TOTAL_WARNING_COUNT,
    };

    /**
     * @brief The errors for the brushless 6 step control loop
     */
    enum class Brushless6StepError {
        /// @brief The rotor position estimator update has failed
        ROTOR_POSITION_ESTIMATOR_UPDATE_FAILURE,
        /// @brief The rotor position estimator get angle has failed
        ROTOR_POSITION_ESTIMATOR_GET_ANGLE_FAILURE,
        /// @brief The bridge duty cycle set call has failed
        BRIDGE_DUTY_CYCLE_SET_FAILURE,
        /// @brief Total number of errors
        TOTAL_ERROR_COUNT,
    };

    /**
     * @brief Initialize the control loop
     * @note This must be called once before calling run
     */
    void init(void);

    /**
     * @brief Run the control loop
     * @param speed The speed to run the control loop at. -1.0f - 1.0f
     * @return The status of the control loop
     * @note The speed parameter is equivalent to the duty cycle of the PWM signal sent to the motor (i.e. the DC voltage in the
     * BLDC 6 step commutation scheme)
     * @attention This function should be called at the desired control loop frequency. For FOC, this is usually after phase
     * current information becomes available, which is at the centre of the inverter PWM cycle. For Trapezodial PWM, this point is
     * at the start of the PWM cycle. The frequency is usually around 15kHz, the same as the switching frequency of the inverter.
     */
    ControlLoop::ControlLoopBaseStatus run(float speed) override;

    /**
     * @brief The detailed status of the brushless 6 step control loop
     */
    using Brushless6StepStatus = ControlLoop::ControlLoopStatus<Brushless6StepError, Brushless6StepWarning>;

    /**
     * @brief Get the detailed status of the control loop
     * @return The status of the control loop
     */
    Brushless6StepStatus get_status() const { return status_; }

   private:
    hwbridge::Bridge3Phase& motor_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator& rotor_position_estimator_;

    Brushless6StepStatus status_;

    hwbridge::Bridge3Phase::phase_command_t phase_command_[hwbridge::Bridge3Phase::NUM_PHASES];
};

}  // namespace control_loop

#endif