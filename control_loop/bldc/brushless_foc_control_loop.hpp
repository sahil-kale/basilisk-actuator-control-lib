#ifndef BRUSHLESS_FOC_CONTROL_LOOP_HPP
#define BRUSHLESS_FOC_CONTROL_LOOP_HPP

#include <stdlib.h>

#include <array>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "foc_util.hpp"
#include "hal_clock.hpp"
#include "math_foc.hpp"
#include "pid.hpp"
#include "rotor_estimator.hpp"

namespace control_loop {

/**
 * @brief A control loop for a brushless motor
 */
class BrushlessFOCControlLoop : public ControlLoop {
   public:
    /**
     * @brief The state of the control loop
     */
    enum class BrushlessFOCControlLoopState {
        /// @brief The control loop is stopped (no PWM output)
        STOP,
        /// @brief The control loop is running
        RUN,
    };

    /**
     * @brief The type of control loop control
     */
    enum class BrushlessFOCControlLoopType {
        /// The control loop is open loop (not relying on rotor position estimation)
        OPEN_LOOP,
        /// The current control loop is in closed loop control
        CLOSED_LOOP,
    };

    /**
     * @brief The parameters used by the FOC control loop
     */
    class BrushlessFocControLoopParams {
       public:
        /**
         * @brief The bandwidth of the current control loop
         */
        float current_control_bandwidth_rad_per_sec;

        /**
         * @brief The phase resistance of the motor (ohms)
         */
        float phase_resistance;
        /**
         * @brief The phase inductance of the motor (henries)
         */
        float phase_inductance;
        /**
         * @brief The flux linkage of the permanent magnet (weber)
         */
        float pm_flux_linkage;

        /**
         * @brief The timeout period for the foc start (us)
         */
        utime_t foc_start_timeout_period_us;
        /**
         * @brief Disable the ki term of the current controller
         * @note Do so if the current controller is unstable in a low bandwidth system, experimentally seems to yield better
         * results
         */
        bool disable_ki;

        /**
         * @brief Converts a generic -1.0 -> 1.0 speed to a iq reference by multiplying by this value by the speed
         */
        float speed_to_iq_gain;  // Converts speed to iq reference

        /**
         * @brief The default d current reference (Amps)
         */
        float i_d_reference_default;

        /**
         * @brief The cutoff frequency of the low pass filter for the current controller (Hz)
         */
        float current_lpf_fc;

        /**
         * @brief The PWM control type to use for the FOC control loop
         */
        BldcFoc::BrushlessFocPwmControlType pwm_control_type;
    };

    /**
     * @brief The parameters for the control loop
     */
    class BrushlessFOCControlLoopParams {
       public:
        /**
         * @brief The FOC control loop parameters
         */
        BrushlessFocControLoopParams foc_params;
        /// @brief The open loop full speed theta velocity (rad/s)
        float open_loop_full_speed_theta_velocity;
        /// @brief The magnitude of the direct voltage vector to apply in open loop mode
        float open_loop_quadrature_voltage;
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
     * @param rotor_position_estimators A pointer array to rotor position estimators to use in C-style array
     * @param num_rotor_position_estimators The number of rotor position estimators
     * @note The secondary rotor position estimator is used when the primary rotor position estimator is not valid
     */
    BrushlessFOCControlLoop(hwbridge::Bridge3Phase& motor, basilisk_hal::HAL_CLOCK& clock,
                            bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                            size_t num_rotor_position_estimators)
        : bridge_(motor),
          clock_(clock),
          rotor_position_estimators_(rotor_position_estimators),
          num_rotor_position_estimators_(num_rotor_position_estimators),
          pid_q_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock),
          pid_d_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock),
          foc_frame_vars_() {}

    /**
     * @brief Initialize the control loop
     * @param params The control loop parameters
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    void init(BrushlessFOCControlLoopParams* params);

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
     * @brief Inputs to the FOC control loop
     */
    class FOCInputs {
       public:
        /// @brief timestamp of the FOC calculation
        utime_t timestamp = 0;
        /// @brief The time since the last FOC calculation (seconds)
        float dt = 0.0f;
        /// @brief The rotor position at the time of the FOC calculation (radians)
        float theta_e = 0.0f;
        /// @brief Whether the rotor position is valid at the time of the FOC calculation
        bool rotor_position_valid = false;

        /// @brief The DC bus voltage at the time of the FOC calculation (Volts)
        float bus_voltage = 0.0f;
        /// @brief Whether the DC bus voltage is valid at the time of the FOC calculation
        bool bus_voltage_valid = false;

        /// @brief Whether the current measurements and derived calculations are valid at the time of the FOC calculation
        bool current_measurements_valid = false;
        /// @brief The alpha/beta current vector at the time of the FOC calculation inferred from the phase currents (A)
        math::alpha_beta_t i_alpha_beta;
        /// @brief The the direct/quadrature current at the time of the FOC calculation (A)
        math::direct_quad_t i_direct_quad;

        /// @brief The desired direct/quadrature current reference at the time of the FOC calculation (A)
        math::direct_quad_t i_direct_quad_ref;
    };

    /**
     * @brief FOC Computation 'Frame' that contains the inputs and outputs of the FOC computation
     */
    class FOCFrameVars {
       public:
        /// @brief The inputs to the FOC computation
        FOCInputs foc_inputs;

        // Outputs
        /// @brief The control loop type that was used in this FOC computation
        BrushlessFOCControlLoopType control_loop_type = BrushlessFOCControlLoopType::OPEN_LOOP;
        /** @brief The  angle that was used for the FOC computation (radians)
         *  @note in open loop, this is the open loop angle, in closed loop, this is simply the same as the rotor position
         */
        float commanded_rotor_theta = 0.0f;
        /// @brief The direct and quadrature voltage vector at the time of the FOC calculation
        math::direct_quad_t V_direct_quad;
        /// @brief The duty cycle computation result of the frame
        BldcFoc::FocDutyCycleResult duty_cycle_result;
    };

    /**
     * @brief Get the FOC frame variables of the most recent control loop iteration
     * @return The FOC frame computation of the most recent control loop iteration
     */
    FOCFrameVars get_foc_frame_computation() const;

    ~BrushlessFOCControlLoop() = default;

   protected:
    /*! \cond PRIVATE */

    BrushlessFOCControlLoopState state_ = BrushlessFOCControlLoopState::STOP;
    hwbridge::Bridge3Phase& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator** rotor_position_estimators_ = nullptr;
    size_t num_rotor_position_estimators_ = 0;
    // Control loop parameters
    BrushlessFOCControlLoopParams* params_ = nullptr;
    BrushlessFOCControlLoopStatus status_;

    // Control loop state variables
    utime_t last_run_time_ = 0;
    float rotor_position_open_loop_start_ = 0.0f;

    // Create 2 PID controllers for the Q and D currents
    pid::PID<float> pid_q_current_;
    pid::PID<float> pid_d_current_;

    // FOC debug variables
    FOCFrameVars foc_frame_vars_;  // Control Loop FOC frame variables - these need to persist for the next control loop iteration
                                   // as the ValphaBeta and VdirectQuad is used for feedforward control

    /**
     * @brief Run the FOC control loop
     * @param foc_inputs The inputs to the FOC control loop
     * @param phase_commands The phase commands to be filled in
     */
    void run_foc(FOCInputs foc_inputs, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]);

    /**
     * @brief Get the desired state of the control loop
     * @param i_q_reference The desired quadrature current reference for the control loop to track
     * @param current_state The current state
     * @todo If required later, make this based on the current magnitude or perhaps another function to 'arm'?
     * @return The desired state of the control loop
     */
    BrushlessFOCControlLoopState get_desired_state(float i_q_reference, const BrushlessFOCControlLoopState current_state);

    /**
     * @brief Get the desired control loop type
     * @param is_any_estimator_valid Whether any of the rotor position estimators are valid
     * @return The desired control loop type
     */
    BrushlessFOCControlLoopType get_desired_control_loop_type(bool is_any_estimator_valid);

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
                                         const BrushlessFOCControlLoopParams* params, math::alpha_beta_t V_alpha_beta,
                                         BrushlessFOCControlLoopStatus& status, float& theta,
                                         bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                                         size_t num_rotor_position_estimators);

    /**
     * @brief Exit a state
     * @param current_state The current state
     * @param desired_state The desired state to exit
     * @return void
     */
    void exit_state(const BrushlessFOCControlLoopState& current_state, const BrushlessFOCControlLoopState& desired_state);

    /**
     * @brief Enter a state
     * @param current_state The current state
     * @param desired_state The desired state to enter
     * @return void
     */
    void enter_state(const BrushlessFOCControlLoopState& current_state, const BrushlessFOCControlLoopState& desired_state);

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
    FOCInputs update_foc_inputs(utime_t current_time_us, utime_t last_run_time_us,
                                bldc_rotor_estimator::ElectricalRotorPosEstimator* rotor_position_estimators[],
                                size_t num_rotor_position_estimators, hwbridge::Bridge3Phase& bridge,
                                BrushlessFOCControlLoopStatus& status, math::alpha_beta_t V_alpha_beta,
                                const BrushlessFOCControlLoopParams* params, math::direct_quad_t i_direct_quad_ref);

    /*! \endcond */
};

}  // namespace control_loop

#endif  // BRUSHLESS_CONTROL_LOOP_HPP
