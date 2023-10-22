#ifndef BRUSHLESS_CONTROL_LOOP_HPP
#define BRUSHLESS_CONTROL_LOOP_HPP

#include <stdlib.h>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
#include "math_foc.hpp"
#include "pid.hpp"
#include "rotor_estimator.hpp"

namespace control_loop {

// Define a brushless foc control loop class that inherits from ControlLoop
class BrushlessControlLoop : public ControlLoop {
   public:
    enum class BrushlessControlLoopState {
        STOP,
        RUN,
    };

    // Define a control loop type
    enum class BrushlessControlLoopType {
        OPEN_LOOP,
        CLOSED_LOOP,
    };

    enum class BrushlessControlLoopCommutationType {
        TRAPEZOIDAL,
        FOC,
    };

    // Define a pwm control type (Sine or Space-Vector)
    enum class BrushlessFocPwmControlType {
        SINE,
        SPACE_VECTOR,
    };

    // Define FOC control-loop specific parameters
    class BrushlessFocControLoopParams {
       public:
        float current_control_bandwidth_rad_per_sec;  // The bandwidth of the current control loop

        float phase_resistance;
        float phase_inductance;
        float pm_flux_linkage;

        utime_t foc_start_timeout_period_us;
        bool disable_ki;  // Disable the ki term of the current controller

        float speed_to_iq_gain;  // Converts speed to iq reference
        float i_d_reference_default;

        float current_lpf_fc;  // The cutoff frequency of the low pass filter for the current controller

        BrushlessFocPwmControlType pwm_control_type;
    };

    class BrushlessControlLoopParams {
       public:
        BrushlessControlLoopCommutationType commutation_type;
        BrushlessFocControLoopParams foc_params;
        float open_loop_full_speed_theta_velocity;  // rad/s
    };

    class BrushlessControlLoopStatus : public ControlLoopStatus {
       public:
        enum class BrushlessControlLoopError {
            NO_ERROR,
            PARAMS_NOT_SET,
        };
        enum class BrushlessControlLoopWarning {
            NO_WARNING,
            CURRENT_CONTROL_NOT_SUPPORTED,
        };
        BrushlessControlLoopStatus() : ControlLoopStatus() {}
        BrushlessControlLoopWarning warning = BrushlessControlLoopWarning::NO_WARNING;
        BrushlessControlLoopError error = BrushlessControlLoopError::NO_ERROR;

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

    // Provide a const getter for the status
    const BrushlessControlLoopStatus& get_status() const { return status_; }

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
     */
    void init(BrushlessControlLoopParams* params);

    /**
     * @brief Run the control loop
     * @param speed The desired speed of the motor (note: this is multiplied by the speed_to_iq_gain)
     * @note: speed is from -1 -> 1
     * @return The status of the control loop
     */
    ControlLoopStatus run(float speed) override;

    /**
     * @brief Run the control loop in current control mode
     * @param i_d_reference The desired d current
     * @param i_q_reference The desired q current
     * @note this function shuold only be used when the control loop control type is FOC
     * @return The status of the control loop
     */
    ControlLoopStatus run_current_control(float i_d_reference, float i_q_reference);

    ~BrushlessControlLoop() = default;

   protected:
    BrushlessControlLoopState state_ = BrushlessControlLoopState::STOP;
    hwbridge::Bridge3Phase& bridge_;
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator& primary_rotor_position_estimator_;
    bldc_rotor_estimator::ElectricalRotorPosEstimator* secondary_rotor_position_estimator_;
    // Control loop parameters
    BrushlessControlLoopParams* params_ = nullptr;
    BrushlessControlLoopStatus status_;

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

    // duty cycles
    float duty_cycle_u_h_ = 0.0f, duty_cycle_v_h_ = 0.0f, duty_cycle_w_h_ = 0.0f;

    // FOC variables
    float i_quadrature_ = 0.0f, i_direct_ = 0.0f, V_quadrature_ = 0.0f, V_direct_ = 0.0f, V_alpha_ = 0.0f, V_beta_ = 0.0f;
    float i_d_reference_ = 0.0f;

    /**
     * @brief Get the desired state of the control loop
     */
    BrushlessControlLoopState get_desired_state(float motor_speed, const BrushlessControlLoopState current_state);

    /**
     * @brief Get the desired control loop type
     * @param is_estimator_valid Whether the rotor position estimator is valid
     * @return The desired control loop type
     */
    BrushlessControlLoopType get_desired_control_loop_type(bool is_estimator_valid);

    /**
     * @brief update the rotor position estimator
     * @param estimator_inputs The inputs to the rotor position estimator
     * @param current_time_us The current time
     * @return void
     */
    void update_rotor_position_estimator(bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs& estimator_inputs,
                                         utime_t current_time_us);

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
    void determine_inverter_duty_cycles_foc(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                            BrushlessFocPwmControlType pwm_control_type,
                                            hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                            hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                            hwbridge::Bridge3Phase::phase_command_t& phase_command_w);

    /**
     * @brief Generate the duty cycles for the 3 phases based on the commutation step
     * @param phase_command array of phase commands to be filled in, u-v-w
     * @param commutation_step the commutation step to generate the duty cycles for
     * @param motor_speed the motor speed to generate the duty cycles for. 0 - 1.0f
     */
    void determine_inverter_duty_cycles_trap(hwbridge::Bridge3Phase::phase_command_t phase_command[3],
                                             Bldc6StepCommutationTypes::commutation_step_t current_commutation_step,
                                             float motor_speed);
};

}  // namespace control_loop

#endif  // BRUSHLESS_CONTROL_LOOP_HPP
