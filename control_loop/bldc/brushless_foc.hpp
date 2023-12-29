#ifndef BRUSHLESS_FOC_HPP
#define BRUSHLESS_FOC_HPP
#include "bridge_3phase.hpp"
#include "math_foc.hpp"
#include "pid.hpp"

namespace control_loop {
namespace BldcFoc {

// Define a pwm control type (Sine or Space-Vector)
enum class BrushlessFocPwmControlType {
    SINE,
    SPACE_VECTOR,
};

// Define a struct to hold the result of the duty cycle computation for FOC, alongside
// other useful values
class FocDutyCycleResult {
   public:
    float duty_cycle_u_h = 0.0f;
    float duty_cycle_v_h = 0.0f;
    float duty_cycle_w_h = 0.0f;

    math::alpha_beta_pair_t V_alpha_beta;
};

/**
 * @brief Determine the duty cycles for the inverter using the FOC algorithm by doing inverse park and vector control algo
 * (inverse clarke or foc)
 * @param theta The rotor angle (radians)
 * @param V_dq The voltage vector in the dq frame
 * @param bus_voltage The bus voltage
 * @param pwm_control_type The type of pwm control to use
 * @param phase_command_u The duty cycle for phase u
 * @param phase_command_v The duty cycle for phase v
 * @param phase_command_w The duty cycle for phase w
 * @note The return duty cycles are between 0.0f and 1.0f, where 0.5 is 50% duty cycle and represents 0V (assumes complementary
 * pwm)
 * @note The max duty cycle is only used by the sine pwm control type. SVPWM will always use map from 0.0f to 1.0f
 *
 * @return The result of the duty cycle computation
 */
FocDutyCycleResult determine_inverter_duty_cycles_foc(float theta, math::dq_pair_t V_dq, float bus_voltage,
                                                      BrushlessFocPwmControlType pwm_control_type,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_w);

/**
 * @brief Convert 3 phase currents to a dq frame
 * @param phase_currents The phase currents
 * @param theta The electrical angle of the rotor (radians)
 * @note theta should be in the range [0, 2*pi) and defined as the angle between the alpha axis and the d axis
 * @return math::dq_pair_t The dq frame current
 */
math::dq_pair_t convert_current_to_dq_frame(const hwbridge::Bridge3Phase::phase_current_t& phase_currents, float theta);

/**
 * @brief Determine the voltage vector for the FOC algorithm (i.e. run the current controllers)
 * @param pi_id The PI controller for the direct current
 * @param pi_iq The PI controller for the quadrature current
 * @param i_q_reference The quadrature current reference
 * @param i_d_reference The direct current reference
 * @param i_dq The dq frame current
 * @param V_dq_ff The feed forward voltage vector
 * @return FocCurrentControllerResult The result of the current controllers
 */
math::dq_pair_t determine_voltage_vector_foc(pid::PID<float>& pi_id, pid::PID<float>& pi_iq, float i_q_reference,
                                             float i_d_reference, math::dq_pair_t i_dq, math::dq_pair_t V_dq_ff);

/**
 * @brief Clamps the given Vd/Vq vector by determining the maximum vector length and scaling the vector if necessary
 * @param V_dq The desired output voltage vector
 * @param bus_voltage The bus voltage
 * @return math::dq_pair_t The clamped Vd/Vq vector
 */
math::dq_pair_t clamp_Vdq_vector(math::dq_pair_t V_dq, float bus_voltage);

class svpwm_duty_cycle {
   public:
    float dutyCycleU = 0.0f;
    float dutyCycleV = 0.0f;
    float dutyCycleW = 0.0f;
};

/**
 * @brief Perform a space vector pulse width modulation on the given alpha/beta voltage values.
 * @param Vd Voltage in the d frame
 * @param Vq Voltage in the q frame
 * @param theta_el The electrical angle of the rotor
 * @param Vbus The bus voltage
 * @return svpwm_duty_cycle The result of the svpwm
 */
svpwm_duty_cycle svpwm(float Vd, float Vq, float theta_el, float Vbus);

class FOCDebugVars {
   public:
    // Inputs
    float theta_e = 0.0f;
    math::dq_pair_t i_dq;

    // Outputs
    math::dq_pair_t V_dq;
    FocDutyCycleResult duty_cycle_result;
};

}  // namespace BldcFoc

}  // namespace control_loop

#endif  // BRUSHLESS_FOC_HPP
