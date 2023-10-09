#ifndef BRUSHLESS_FOC_CONTROL_LOOP_HPP
#define BRUSHLESS_FOC_CONTROL_LOOP_HPP

// forward declare when in unit test
#if defined(UNIT_TEST) || defined(DEBUG)
class BrushlessFocControlLoopTest;
#endif

#include <stdlib.h>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "math_foc.hpp"
#include "pid.hpp"
#include "rotor_estimator.hpp"

namespace control_loop {

// Define a brushless foc control loop class that inherits from ControlLoop
class BrushlessFocControlLoop : public ControlLoop {
   public:
    enum class BrushlessFocControlLoopState {
        NOT_INITIALIZED,
        STOP,
        RUN,
    };

    // Define a control loop type
    enum class BrushlessFocControlLoopType {
        OPEN_LOOP,
        CLOSED_LOOP,
    };

    // Define a pwm control type (Sine or Space-Vector)
    enum class BrushlessFocPwmControlType {
        SINE,
        SPACE_VECTOR,
    };

    // Define control-loop specific parameters
    class BrushlessFocControLoopParams {
       public:
        float kp_q_current;
        float ki_q_current;
        float kd_q_current;
        float kp_d_current;
        float ki_d_current;
        float kd_d_current;
        utime_t foc_start_timeout_period_us;

        float speed_to_iq_gain;  // Converts speed to iq reference
        float i_d_reference;

        float open_loop_full_speed_theta_velocity;  // rad/s

        BrushlessFocPwmControlType pwm_control_type;
    };

    BrushlessFocControlLoop(hwbridge::Bridge3Phase& motor, basilisk_hal::HAL_CLOCK& clock,
                            bldc_rotor_estimator::BldcElectricalRotorPositionEstimator& rotor_position_estimator)
        : bridge_(motor), clock_(clock), rotor_position_estimator_(rotor_position_estimator) {}

    /**
     * @brief Initialize the control loop
     * @param params The control loop parameters
     */
    void init(BrushlessFocControLoopParams* params);

    /**
     * @brief Run the control loop
     * @param speed The desired speed of the motor (note: this is multiplied by the speed_to_iq_gain)
     * @note: speed is from -1 -> 1
     */
    void run(float speed) override;

    ~BrushlessFocControlLoop() = default;

   protected:
    BrushlessFocControlLoopState state_ = BrushlessFocControlLoopState::NOT_INITIALIZED;
    hwbridge::Bridge3Phase& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::BldcElectricalRotorPositionEstimator& rotor_position_estimator_;
    // Control loop parameters
    BrushlessFocControLoopParams* params_ = nullptr;

    // Control loop state variables
    utime_t time_at_start_ = 0;
    utime_t last_run_time_ = 0;
    float rotor_position_open_loop_start_ = 0.0f;
    float motor_speed_, rotor_position_ = 0;

    BrushlessFocControlLoopType control_loop_type_ = BrushlessFocControlLoopType::OPEN_LOOP;

    // Create 2 PID controllers for the Q and D currents
    pid::PID<float> pid_q_current_{0.0, 0, 0, 0, 0, 0};
    pid::PID<float> pid_d_current_{0.0, 0, 0, 0, 0, 0};
    float desired_rotor_angle_open_loop_ = 0.0f;

    // duty cycles
    float duty_cycle_u_h_, duty_cycle_v_h_, duty_cycle_w_h_ = 0.0f;

    // FOC variables
    float i_quadrature_, i_direct_, V_quadrature_, V_direct_, V_alpha_, V_beta_ = 0.0f;

    /**
     * @brief Get the desired state of the control loop
     */
    BrushlessFocControlLoopState get_desired_state(float motor_speed, const BrushlessFocControlLoopState current_state);

    /**
     * @brief Get the desired control loop type
     * @param is_estimator_valid Whether the rotor position estimator is valid
     * @return The desired control loop type
     */
    BrushlessFocControlLoopType get_desired_control_loop_type(bool is_estimator_valid);

    /**
     * @brief Determine the duty cycles for the inverter using the FOC algorithm by doing inverse park and vector control algo
     * (inverse clarke or foc)
     * @param theta The rotor angle (radians)
     * @param Vdirect The alpha component of the voltage vector
     * @param Vquardature The beta component of the voltage vector
     * @param bus_voltage The bus voltage
     * @param phase_command_u The duty cycle for phase u
     * @param phase_command_v The duty cycle for phase v
     * @param phase_command_w The duty cycle for phase w
     */
    void determine_inverter_duty_cycles(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                        BrushlessFocPwmControlType pwm_control_type,
                                        hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                        hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                        hwbridge::Bridge3Phase::phase_command_t& phase_command_w);

#if defined(UNIT_TEST) || defined(DEBUG)
    friend class BrushlessFocControlLoopTest;
#endif
};

}  // namespace control_loop

#endif  // BRUSHLESS_FOC_CONTROL_LOOP_HPP
