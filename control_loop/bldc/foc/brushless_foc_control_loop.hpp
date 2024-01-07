#ifndef BRUSHLESS_FOC_CONTROL_LOOP_HPP
#define BRUSHLESS_FOC_CONTROL_LOOP_HPP

#include <stdlib.h>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "foc_controller.hpp"
#include "foc_util.hpp"
#include "hal_clock.hpp"
#include "phase_inductance_estimator.hpp"
#include "phase_resistance_estimator.hpp"
#include "pid.hpp"
#include "rotor_estimator.hpp"

namespace control_loop {

using namespace BldcFoc;

/**
 * @brief A control loop for a brushless motor
 */
class BrushlessFOCControlLoop : public ControlLoop {
   public:
    /**
     * @brief The state of the control loop
     */
    enum class State {
        /// @brief The control loop is stopped (no PWM output)
        STOP,
        /** @brief The control loop is calibrating the motor - it may move slightly during this time
         *  @details The calibration state is entered when phase resistance or inductance are not valid.
         */
        CALIBRATION,
        /// @brief The control loop is running
        RUN,
    };

    /**
     * @brief The parameters used by the FOC control loop
     */
    class BrushlessFocControLoopParams {
       public:
        /**
         * @brief The bandwidth of the current control loop
         */
        float current_control_bandwidth_rad_per_sec = 0.0f;

        /// @brief The phase resistance of the motor (ohms)
        float phase_resistance = 0.0f;
        /// @brief Whether or not the phase resistance is valid
        bool phase_resistance_valid = false;
        /// @brief The phase inductance of the motor (henries)
        float phase_inductance = 0.0f;
        /// @brief Whether or not the phase inductance is valid
        bool phase_inductance_valid = false;
        /**
         * @brief The flux linkage of the permanent magnet (weber)
         */
        float pm_flux_linkage = 0.0f;

        /**
         * @brief Disable the ki term of the current controller
         * @note Do so if the current controller is unstable in a low bandwidth system, experimentally seems to yield better
         * results
         */
        bool disable_ki = false;

        /**
         * @brief Converts a generic -1.0 -> 1.0 speed to a iq reference by multiplying by this value by the speed
         */
        float speed_to_iq_gain = 0.0f;  // Converts speed to iq reference

        /**
         * @brief The default d current reference (Amps)
         */
        float i_d_reference_default = 0.0f;

        /**
         * @brief The PWM control type to use for the FOC control loop
         */
        BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;
    };

    /**
     * @brief The parameters for the control loop
     */
    class Params {
       public:
        /**
         * @brief The FOC control loop parameters
         */
        BrushlessFocControLoopParams foc_params;
        /// @brief The open loop full speed theta velocity (rad/s)
        float open_loop_theta_velocity = 0.0f;
        /// @brief The magnitude of the direct voltage vector to apply in open loop mode
        float open_loop_quadrature_voltage = 0.0f;

        /// @brief The phase inductance estimator parameters
        hwbridge::PhaseInductanceEstimatorController::Params phase_inductance_estimator_params;
        /// @brief The phase resistance estimator parameters
        hwbridge::PhaseResistanceEstimatorController::Params phase_resistance_estimator_params;
    };

    /**
     * @brief The error conditions for the brushless control loop
     */
    enum class BrushlessFOCControlLoopError : uint8_t {
        /// The control loop parameters are not set (the pointer is null)
        PARAMS_NOT_SET,
        /// The bus voltage read failed while running the control loop with FOC
        BUS_VOLTAGE_READ_FAILURE,
        /// The phase duty cycle set command failed
        PHASE_COMMAND_FAILURE,
        /// The phase current read from the bridge failed
        PHASE_CURRENT_READ_FAILURE,
        /// The phase inductance estimator failed
        PHASE_INDUCTANCE_ESTIMATOR_FAILURE,
        /// The phase resistance estimator failed
        PHASE_RESISTANCE_ESTIMATOR_FAILURE,
        /// The total number of errors
        TOTAL_ERROR_COUNT,
    };
    /**
     * @brief The warning conditions for the brushless control loop
     */
    enum class BrushlessFOCControlLoopWarning : uint8_t {
        /// The rotor estimator update failed
        ROTOR_ESTIMATOR_UPDATE_FAILURE,
        /// There is no valid rotor position estimator (primary or secondary)
        NO_VALID_ROTOR_POSITION_ESTIMATOR,
        /// The total number of warnings
        TOTAL_WARNING_COUNT,
    };

    /// @brief Detailed status of the FOC control loop
    using BrushlessFOCControlLoopStatus = ControlLoopStatus<BrushlessFOCControlLoopError, BrushlessFOCControlLoopWarning>;

    /**
     * @brief Get the status of the control loop
     * @return The status of the control loop
     */
    BrushlessFOCControlLoopStatus get_status() const { return status_; }

    /**
     * @brief Construct a new BrushlessFOCControlLoop object
     * @param motor The bridge on which a singular BLDC/PMSM motor is connected
     * @param clock The clock to use for the control loop
     * @param rotor_position_estimators A pointer array to rotor position estimators to use in C-style array. The angle estimate
     * used by the controller is the lowest-indexed valid rotor position estimator in the array
     * @param num_rotor_position_estimators The number of rotor position estimators
     */
    BrushlessFOCControlLoop(hwbridge::Bridge3Phase& motor, basilisk_hal::HAL_CLOCK& clock,
                            bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                            size_t num_rotor_position_estimators)
        : bridge_(motor),
          clock_(clock),
          rotor_position_estimators_(rotor_position_estimators),
          num_rotor_position_estimators_(num_rotor_position_estimators),
          foc_controller_(clock),
          phase_inductance_estimator_(clock, hwbridge::PhaseInductanceEstimatorController::Params()),
          phase_resistance_estimator_(clock, hwbridge::PhaseResistanceEstimatorController::Params()),
          foc_frame_vars_() {}

    /**
     * @brief Initialize the control loop
     * @param params The control loop parameters
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    void init(Params* params);

    /**
     * @brief Run the control loop
     * @param speed The desired speed of the motor (note: this is multiplied by the speed_to_iq_gain)
     * @note: speed is from -1 -> 1
     * @return The status of the control loop
     * @attention This function should be called at the desired control loop frequency. For FOC, this is usually after phase
     * current information becomes available, which is at the centre of the inverter PWM cycle. For Trapezodial PWM, this point is
     * at the start of the PWM cycle. The frequency is usually around 15kHz, the same as the switching frequency of the inverter.
     */
    ControlLoop::ControlLoopBaseStatus run(float speed) override;

    /**
     * @brief Run the control loop in current control mode
     * @param i_d_reference The desired d current
     * @param i_q_reference The desired q current
     * @note This function shuold only be used when the control loop control type is FOC
     * @return The status of the control loop
     * @attention This function should be called at the FOC loop frequency at or after phase current information becomes available
     * - this is usually around 15kHz and will be in the middle of the PWM cycle (phase is dependent on HW current sensing
     * implementation)
     */
    ControlLoop::ControlLoopBaseStatus run_current_control(float i_d_reference, float i_q_reference);

    /**
     * @brief Get the FOC frame variables of the most recent control loop iteration
     * @return The FOC frame computation of the most recent control loop iteration
     */
    FOCController::FOCFrameVars get_foc_frame_computation() const;

    ~BrushlessFOCControlLoop() = default;

   protected:
    /*! \cond PRIVATE */

    State state_ = State::STOP;
    hwbridge::Bridge3Phase& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator** rotor_position_estimators_ = nullptr;
    size_t num_rotor_position_estimators_ = 0;
    // Control loop parameters
    Params* params_ = nullptr;
    BrushlessFOCControlLoopStatus status_;

    // Controllers
    FOCController foc_controller_;
    hwbridge::PhaseInductanceEstimatorController phase_inductance_estimator_;
    hwbridge::PhaseResistanceEstimatorController phase_resistance_estimator_;

    // Control loop state variables
    utime_t last_run_time_ = 0;

    // FOC debug variables
    FOCController::FOCFrameVars
        foc_frame_vars_;  // Control Loop FOC frame variables - these need to persist for the next control loop iteration
                          // as the ValphaBeta and VdirectQuad is used for feedforward control

    /**
     * @brief Get the desired state of the control loop
     * @param i_q_reference The desired quadrature current reference for the control loop to track
     * @param current_state The current state
     * @param params The control loop parameters (used to determine if calibration is required)
     * @param status The status of the control loop
     * @todo If required later, make this based on the current magnitude or perhaps another function to 'arm'?
     * @return The desired state of the control loop
     */
    State get_desired_state(float i_q_reference, const State current_state, const Params& params,
                            const BrushlessFOCControlLoopStatus& status);

    /**
     * @brief update the rotor position estimator
     * @param estimator_inputs The inputs to the rotor position estimator
     * @param current_time_us The current time
     * @param phase_currents The phase currents
     * @param params The control loop parameters
     * @param V_alpha_beta The alpha/beta voltage vector
     * @param status The status of the control loop
     * @param theta The estimated rotor position (updated by this function)
     * @param rotor_position_estimators The rotor position estimators in array form
     * @param num_rotor_position_estimators The number of rotor position estimators
     * @note This function updates the rotor position estimator and updates the status of the control loop accordingly
     * @note This function takes in several rotor position estimators and updates the angle with the first valid one
     * @attention The status should be checked to ensure there is a valid rotor position to assert that the theta value is valid
     * @return void
     */
    void update_rotor_position_estimator(bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs& estimator_inputs,
                                         utime_t current_time_us, hwbridge::Bridge3Phase::phase_current_t phase_currents,
                                         const Params* params, math::alpha_beta_t V_alpha_beta,
                                         BrushlessFOCControlLoopStatus& status, float& theta,
                                         bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                                         size_t num_rotor_position_estimators);

    /**
     * @brief Exit a state
     * @param current_state The current state
     * @param desired_state The desired state to exit
     * @return void
     */
    void exit_state(const State& current_state, const State& desired_state);

    /**
     * @brief Enter a state
     * @param current_state The current state
     * @param desired_state The desired state to enter
     * @return void
     */
    void enter_state(const State& current_state, const State& desired_state);

    /**
     * @brief Run the calibration routine
     * @param FOC_inputs The FOC inputs (technically not fully required, but using it since it contains everything we'd possibly
     * need)
     * @param phase_commands The phase commands
     * @return void
     */
    void run_calibration(FOCController::FOCInputs inputs, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]);

    /**
     * @brief Reset the position estimators
     * @param rotor_position_estimators The rotor position estimators to reset
     * @param num_rotor_position_estimators The number of rotor position estimators
     * @return Whether all of the position estimators were successfully reset
     */
    bool reset_position_estimators(bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                                   size_t num_rotor_position_estimators);

    /**
     * @brief Update the FOC inputs
     * @param current_time_us The current time
     * @param last_run_time_us The last time the control loop was run
     * @param rotor_position_estimators The rotor position estimators
     * @param num_rotor_position_estimators The number of rotor position estimators
     * @param bridge The bridge from which to get the phase currents
     * @param status The status of the control loop (updated by this function dependant on errors)
     * @param V_alpha_beta The alpha/beta voltage vector from the previous iteration (used for estimator feedforward in sensorless
     * applications)
     * @param params The control loop parameters
     * @param i_direct_quad_ref The direct/quadrature current reference
     * @return The FOC inputs for the current control frame
     */
    FOCController::FOCInputs update_foc_inputs(utime_t current_time_us, utime_t last_run_time_us,
                                               bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                                               size_t num_rotor_position_estimators, hwbridge::Bridge3Phase& bridge,
                                               BrushlessFOCControlLoopStatus& status, math::alpha_beta_t V_alpha_beta,
                                               const Params* params, math::direct_quad_t i_direct_quad_ref);

    /*! \endcond */
};

}  // namespace control_loop

#endif  // BRUSHLESS_CONTROL_LOOP_HPP
