#ifndef BRUSHLESS_CONTROL_LOOP_HPP
#define BRUSHLESS_CONTROL_LOOP_HPP

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
class BrushlessControlLoop : public ControlLoop {
   public:
    /**
     * @brief The state of the control loop
     */
    enum class BrushlessControlLoopState {
        STOP,
        RUN,
    };

    /**
     * @brief The type of control loop control
     */
    enum class BrushlessControlLoopType {
        /// The control loop is open loop (not relying on rotor position estimation)
        OPEN_LOOP,
        /// The control loop is closed loop control
        CLOSED_LOOP,
    };

    /**
     * @brief The type of commutation to use
     */
    enum class BrushlessControlLoopCommutationType {
        /// 6 step commutation (requires voltage sensing or sensored operation)
        TRAPEZOIDAL,
        /// Field oriented control (requires current sensing)
        FOC,
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
    class BrushlessControlLoopParams {
       public:
        /**
         * @brief The commutation type to use *
         */
        BrushlessControlLoopCommutationType commutation_type;
        /**
         * @brief The FOC control loop parameters
         */
        BrushlessFocControLoopParams foc_params;
        /**
         * @brief Speed at which the open loop control loop is at full speed (rad/s)
         */
        float open_loop_full_speed_theta_velocity;
    };

    /**
     * @brief The error conditions for the brushless control loop
     */
    enum class BrushlessControlLoopError : uint8_t {
        /// The control loop parameters are not set (the pointer is null)
        PARAMS_NOT_SET,
        /// The rotor estimation failed
        ROTOR_ESTIMATION_FAILED,
        /// There is no valid rotor position estimator (primary or secondary)
        NO_VALID_ROTOR_POSITION_ESTIMATOR,
        /// The current control mode is not supported as the control loop type is not FOC
        CURRENT_CONTROL_NOT_SUPPORTED,
        /// The bus voltage read failed while running the control loop with FOC
        BUS_VOLTAGE_READ_FAILURE,
        /// The phase duty cycle set command failed
        PHASE_COMMAND_FAILURE,
        /// The total number of errors
        TOTAL_ERROR_COUNT,
    };
    /**
     * @brief The warning conditions for the brushless control loop
     */
    enum class BrushlessControlLoopWarning : uint8_t {
        /// The primary rotor estimator is not valid
        PRIMARY_ROTOR_ESTIMATOR_NOT_VALID,
        /// The rotor estimator update failed
        ROTOR_ESTIMATOR_UPDATE_FAILURE,
        /// The total number of warnings
        TOTAL_WARNING_COUNT,
    };

    /**
     * @brief Get the status of the control loop
     * @return The status of the control loop
     */
    const ControlLoopStatus<BrushlessControlLoopError, BrushlessControlLoopWarning>& get_status() const { return status_; }

    /**
     * @brief Construct a new BrushlessControlLoop object
     * @param motor The bridge on which a singular BLDC/PMSM motor is connected
     * @param clock The clock to use for the control loop
     * @param rotor_position_estimator The rotor position estimator to use
     * @param secondary_rotor_position_estimator The secondary rotor position estimator to use
     * @note The secondary rotor position estimator is used when the primary rotor position estimator is not valid
     */
    BrushlessControlLoop(hwbridge::Bridge3Phase& motor, basilisk_hal::HAL_CLOCK& clock,
                         bldc_rotor_estimator::ElectricalRotorPosEstimator& rotor_position_estimator,
                         bldc_rotor_estimator::ElectricalRotorPosEstimator* secondary_rotor_position_estimator = nullptr)
        : bridge_(motor),
          clock_(clock),
          primary_rotor_position_estimator_(rotor_position_estimator),
          secondary_rotor_position_estimator_(secondary_rotor_position_estimator),
          pid_q_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock),
          pid_d_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock) {}

    /**
     * @brief Initialize the control loop
     * @param params The control loop parameters
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    void init(BrushlessControlLoopParams* params);

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
     * @brief Get the FOC debug variables of the most recent control loop iteration
     * @return The FOC debug variables
     */
    BldcFoc::FOCDebugVars get_foc_debug_vars() const;

    ~BrushlessControlLoop() = default;

   protected:
    /*! \cond PRIVATE */

    BrushlessControlLoopState state_ = BrushlessControlLoopState::STOP;
    hwbridge::Bridge3Phase& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator& primary_rotor_position_estimator_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator* secondary_rotor_position_estimator_;
    // Control loop parameters
    BrushlessControlLoopParams* params_ = nullptr;
    ControlLoopStatus<BrushlessControlLoopError, BrushlessControlLoopWarning> status_;

    // Control loop state variables
    utime_t time_at_start_ = 0;
    utime_t last_run_time_ = 0;
    float rotor_position_open_loop_start_ = 0.0f;
    float motor_speed_ = 0.0f;
    float rotor_position_ = 0;

    BrushlessControlLoopType control_loop_type_ = BrushlessControlLoopType::OPEN_LOOP;

    // Create 2 PID controllers for the Q and D currents
    pid::PID<float> pid_q_current_;
    pid::PID<float> pid_d_current_;
    float desired_rotor_angle_open_loop_ = 0.0f;

    // FOC variables
    math::alpha_beta_t i_alpha_beta_;    // The Ialpha and Ibeta current
    math::direct_quad_t i_direct_quad_;  // Iq and Id current
    math::direct_quad_t V_direct_quad_;  // Vq and Vd voltage
    math::alpha_beta_t V_alpha_beta_;    // The Valpha and Vbeta voltage
    float i_d_reference_ = 0.0f;         // The desired d current

    // FOC debug variables
    BldcFoc::FOCDebugVars foc_debug_vars_;  // Control Loop FOC debug variables

    /**
     * @brief Get the desired state of the control loop
     * @param motor_speed The desired speed of the motor (note: this is multiplied by the speed_to_iq_gain)
     * @param current_state The current state
     * @return The desired state of the control loop
     */
    BrushlessControlLoopState get_desired_state(float motor_speed, const BrushlessControlLoopState current_state);

    /**
     * @brief Get the desired control loop type
     * @param is_primary_estimator_valid Whether the primary rotor estimator is valid
     * @param is_secondary_estimator_valid Whether the secondary rotor estimator is valid
     * @return The desired control loop type
     */
    BrushlessControlLoopType get_desired_control_loop_type(bool is_primary_estimator_valid, bool is_secondary_estimator_valid);

    /**
     * @brief update the rotor position estimator
     * @param estimator_inputs The inputs to the rotor position estimator
     * @param current_time_us The current time
     * @return void
     */
    void update_rotor_position_estimator(bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs& estimator_inputs,
                                         utime_t current_time_us);

    /**
     * @brief Exit a state
     * @param current_state The current state
     * @param desired_state The desired state to exit
     * @return void
     */
    void exit_state(const BrushlessControlLoopState& current_state, const BrushlessControlLoopState& desired_state);

    /**
     * @brief Enter a state
     * @param current_state The current state
     * @param desired_state The desired state to enter
     * @return void
     */
    void enter_state(const BrushlessControlLoopState& current_state, const BrushlessControlLoopState& desired_state);

    /**
     * @brief Run the FOC control loop
     * @param motor_speed The desired speed of the motor (note: this is multiplied by the speed_to_iq_gain)
     * @param current_time The current time
     * @param last_run_time The last time the control loop was run
     * @param phase_currents The phase current
     * @param phase_commands The phase commands to be filled in
     */
    void run_foc(float speed, utime_t current_time_us, utime_t last_run_time,
                 hwbridge::Bridge3Phase::phase_current_t phase_currents,
                 hwbridge::Bridge3Phase::phase_command_t phase_commands[3]);

    /**
     * @brief Run the trap control loop
     * @param motor_speed The desired speed of the motor to scale the duty cycles by
     * @param phase_commands The phase commands to be filled in
     */
    void run_trap(float speed, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]);

    /*! \endcond */
};

}  // namespace control_loop

#endif  // BRUSHLESS_CONTROL_LOOP_HPP
