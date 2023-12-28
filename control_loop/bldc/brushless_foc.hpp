#ifndef BRUSHLESS_FOC_HPP
#define BRUSHLESS_FOC_HPP
#include "bridge_3phase.hpp"

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
    FocDutyCycleResult() : duty_cycle_u_h(0.0f), duty_cycle_v_h(0.0f), duty_cycle_w_h(0.0f), V_alpha(0.0f), V_beta(0.0f) {}

    float duty_cycle_u_h;
    float duty_cycle_v_h;
    float duty_cycle_w_h;

    float V_alpha;
    float V_beta;
};

/**
 * @brief Determine the duty cycles for the inverter using the FOC algorithm by doing inverse park and vector control algo
 * (inverse clarke or foc)
 * @param theta The rotor angle (radians)
 * @param Vdirect The alpha component of the voltage vector
 * @param Vquardature The beta component of the voltage vector
 * @param bus_voltage The bus voltage
 * @param pwm_control_type The type of pwm control to use
 * @param phase_command_u The duty cycle for phase u
 * @param phase_command_v The duty cycle for phase v
 * @param phase_command_w The duty cycle for phase w
 * @note The return duty cycles are between 0.0f and 1.0f, where 0.5 is 50% duty cycle and represents 0V (assumes complementary
 * pwm)
 * @note The max duty cycle is only used by the sine pwm control type. SVPWM will always use map from 0.0f to 1.0f
 *
 * @return The
 */
FocDutyCycleResult determine_inverter_duty_cycles_foc(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                                      BrushlessFocPwmControlType pwm_control_type,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_w);

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
    float i_direct = 0.0f;
    float i_quadrature = 0.0f;
    // Note: it is possible to calculate i_alpha and i_beta from i_direct and i_quadrature,
    // but we want to capture the actual values we calculated.
    float i_alpha = 0.0f;
    float i_beta = 0.0f;

    // Outputs
    float duty_cycle_u_h = 0.0f;
    float duty_cycle_v_h = 0.0f;
    float duty_cycle_w_h = 0.0f;
    float V_direct = 0.0f;
    float V_quadrature = 0.0f;
    float V_alpha = 0.0f;
    float V_beta = 0.0f;
    float d_voltage_delta = 0.0f;
    float q_voltage_delta = 0.0f;
};

}  // namespace BldcFoc

}  // namespace control_loop

#endif  // BRUSHLESS_FOC_HPP
