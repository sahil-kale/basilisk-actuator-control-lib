#ifndef BRUSHLESS_FOC_HPP
#define BRUSHLESS_FOC_HPP
#include "bridge_3phase.hpp"
#include "math_foc.hpp"

namespace control_loop {
namespace BldcFoc {

// Define a pwm control type (Sine or Space-Vector)
enum class BrushlessFocPwmControlType {
    // Sine PWM control
    SINE,
    // Space-Vector PWM control
    SPACE_VECTOR,
};

/**
 * @brief The results of converting a Vdq voltage vector to duty cycles with a given pwm control type
 * @note The PWM control type is assumed to be known by the caller, as the duty cycles depend on the PWM control type and can vary
 * but are not included in this frame
 */
class FocDutyCycleResult {
   public:
    /**
     * @brief The duty cycle for phase u
     */
    float duty_cycle_u_h = 0.0f;
    /**
     * @brief The duty cycle for phase v
     */
    float duty_cycle_v_h = 0.0f;
    /**
     * @brief The duty cycle for phase w
     */
    float duty_cycle_w_h = 0.0f;

    /**
     * @brief The alpha/beta voltage vector that was used to determine the duty cycles
     */
    math::alpha_beta_t V_alpha_beta;
};

/**
 * @brief Clamp the given DQ voltage vector to the given bus voltage
 * @param V_dq The voltage vector to clamp
 * @param V_bus The bus voltage
 * @return math::direct_quad_t The clamped voltage vector
 * @note The voltage vector magnitude is limited to be a max of 3/4 of the bus voltage due to how much maximum voltage
 * differential can be provided and scales the D/Q components accordingly
 */
math::direct_quad_t clamp_Vdq(math::direct_quad_t V_dq, float V_bus);

/**
 * @brief Determine the duty cycles for the inverter using the FOC algorithm by doing inverse park and vector control algo
 * (inverse clarke or foc)
 * @param theta The rotor angle (radians)
 * @param Vdirect The alpha component of the voltage vector (phase to neutral)
 * @param Vquardature The beta component of the voltage vector (phase to neutral)
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

/**
 * @brief Perform a sine pulse width modulation on the given alpha/beta voltage values.
 * @param V_alpha_beta The voltage vector to create the duty cycles for
 * @param Vbus The bus voltage to use when creating the duty cycles for phase-to-neutral voltages
 * @param max_duty_cycle The maximum duty cycle to use when creating the duty cycles for phase-to-neutral voltages
 * @return abc_t The result of the sine pwm duty cycle calculation (0.0f to 1.0f)
 */
math::abc_t sine_pwm(math::alpha_beta_t V_alpha_beta, float Vbus, float max_duty_cycle = 1.0f);

/**
 * @brief Perform a space vector pulse width modulation on the given alpha/beta voltage values based on the method proposed in
 * https://www2.ece.ohio-state.edu/ems/PowerConverter/SpaceVector_PWM_Inverter.pdf
 * @param V_alpha_beta The voltage vector to create the duty cycles for
 * @param Vbus The bus voltage to clamp the voltage vector to
 * @return abc_t The result of the svpwm duty cycle calculation
 */
math::abc_t svpwm(math::alpha_beta_t V_alpha_beta, float Vbus);

/**
 * @brief Returns the sextant (sector) of the given voltage vector in the alpha/beta frame
 * @param V_alpha_beta The voltage vector to determine the sector of
 * @return uint8_t The sector of the voltage vector
 */
uint8_t svm_sector(math::alpha_beta_t V_alpha_beta);

/**
 * @brief FOC Computation 'Frame' that can be used for debugging
 */
class FOCDebugVars {
   public:
    // Inputs
    /**
     * @brief The timestamp of the FOC calculation. 0 is invalid
     */
    utime_t timestamp = 0;
    /**
     * @brief The electrical theta (radians) at the time of the FOC calculation
     */
    float theta_e = 0.0f;  // The electrical theta (radians)
    /**
     * @brief The Idq current vector at the time of the FOC calculation inferred from the phase currents
     */
    math::direct_quad_t i_direct_quad;

    /**
     * @brief The alpha/beta current vector at the time of the FOC calculation inferred from the phase currents
     * @note This is the same as i_direct_quad, but rotated by theta_e. These are redundant, but useful for debugging if the
     * transforms are off
     */
    math::alpha_beta_t i_alpha_beta;  // The alpha/beta current vector

    // Outputs
    /**
     * @brief The direct and quadrature voltage vector at the time of the FOC calculation
     */
    math::direct_quad_t V_direct_quad;
    /**
     * @brief The duty cycle result of the FOC calculation
     */
    FocDutyCycleResult duty_cycle_result;  // The duty cycle result
};

}  // namespace BldcFoc

}  // namespace control_loop

#endif  // BRUSHLESS_FOC_HPP
