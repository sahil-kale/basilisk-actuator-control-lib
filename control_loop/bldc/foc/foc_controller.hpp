#ifndef FOC_CONTROLLER_HPP
#define FOC_CONTROLLER_HPP
#include "foc_util.hpp"
#include "hal_clock.hpp"
#include "pid.hpp"
namespace control_loop {
namespace BldcFoc {

/**
 * @brief Controller for a FOC control scheme - acts on FOC inputs and returns duty cycles
 * @note The controller implements both open-loop voltage and closed-loop current control. The former mode is activated when the
 * theta estimate is considered to be invalid
 */
class FOCController {
   public:
    /**
     * @brief The type of control loop control
     */
    enum class ControlLoopType {
        /// The control loop is open loop (not relying on rotor position estimation)
        OPEN_LOOP,
        /// The current control loop is in closed loop control
        CLOSED_LOOP,
    };

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

        /// @brief The open loop theta velocity at the time of the FOC calculation if operating in open loop (radians/second)
        float open_loop_theta_velocity = 0.0f;
        /// @brief The open loop voltage at the time of the FOC calculation if operating in open loop (Volts)
        float open_loop_quadrature_voltage = 0.0f;

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

        /// @brief The type of PWM used to compute the duty cycles
        BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;
    };

    /**
     * @brief Create a new FOC controller object
     * @param clock The clock to use for the FOC controller's PID controllers
     * @note The FOC input's dt is used only for the open-loop angle advance, and is not used for the PID controllers
     */
    explicit FOCController(basilisk_hal::HAL_CLOCK& clock)
        : clock_(clock),
          pid_direct_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock),
          pid_quadrature_current_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, clock) {}

    /**
     * @brief Initialize (or reset) the FOC controller
     * @param kp The proportional gain for the current control loops
     * @param ki The integral gain for the current control loops
     * @note This must be called before the FOC controller is used
     * @note This uses the same gains for both the direct and quadrature current control loops
     */
    void init(float kp, float ki);

    /**
     * @brief FOC Computation 'Frame' that contains the inputs and outputs of the FOC computation
     */
    class FOCFrameVars {
       public:
        /// @brief The inputs to the FOC computation
        FOCInputs foc_inputs;

        // Outputs
        /// @brief The control loop type that was used in this FOC computation
        ControlLoopType control_loop_type = ControlLoopType::OPEN_LOOP;
        /** @brief The  angle that was used for the FOC computation (radians)
         *  @note In open loop, this is the open loop angle, in closed loop, this is simply the same as the rotor position
         */
        float commanded_rotor_theta = 0.0f;
        /// @brief The direct and quadrature voltage vector at the time of the FOC calculation
        math::direct_quad_t V_direct_quad;
        /// @brief The duty cycle computation result of the frame
        FocDutyCycleResult duty_cycle_result;
    };

    /**
     * @brief Run the FOC control loop
     * @param foc_inputs The inputs to the FOC control loop
     * @param phase_commands The phase commands to be filled in
     * @return FOCFrameVars The FOC frame variables
     */
    FOCFrameVars run_foc(FOCController::FOCInputs foc_inputs, hwbridge::Bridge3Phase::phase_command_t phase_commands[3]);

   protected:
    /*! \cond PRIVATE */
    FOCFrameVars foc_frame_vars_;
    basilisk_hal::HAL_CLOCK& clock_;

    // PID controllers for the direct and quadrature current control loops
    pid::PID<float> pid_direct_current_;
    pid::PID<float> pid_quadrature_current_;

    /**
     * @brief Get the desired control loop type
     * @param is_any_estimator_valid Whether any of the rotor position estimators are valid
     * @return The desired control loop type
     */
    FOCController::ControlLoopType get_desired_control_loop_type(bool is_any_estimator_valid);

    /*! \endcond */
};

}  // namespace BldcFoc
}  // namespace control_loop

#endif  // FOC_CONTROLLER_HPP