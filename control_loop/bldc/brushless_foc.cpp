#include "brushless_foc.hpp"

#include <cmath>

#include "math_foc.hpp"
#include "math_util.hpp"

namespace control_loop {
namespace BldcFoc {

FocDutyCycleResult determine_inverter_duty_cycles_foc(float theta, math::dq_pair_t V_dq, float bus_voltage,
                                                      BrushlessFocPwmControlType pwm_control_type,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_w) {
    const float max_duty_cycle = 1.0f;
    // Create the result
    FocDutyCycleResult result;
    // Do an inverse Park transform
    math::alpha_beta_pair_t inverse_park_transform = math::inverse_park_transform(V_dq.direct, V_dq.quadrature, theta);
    float duty_cycle_u_h = 0.0f;
    float duty_cycle_v_h = 0.0f;
    float duty_cycle_w_h = 0.0f;

    switch (pwm_control_type) {
        case BldcFoc::BrushlessFocPwmControlType::SPACE_VECTOR: {
            svpwm_duty_cycle duty_cycles = svpwm(V_dq.direct, V_dq.quadrature, theta, bus_voltage);
            duty_cycle_u_h = duty_cycles.dutyCycleU;
            duty_cycle_v_h = duty_cycles.dutyCycleV;
            duty_cycle_w_h = duty_cycles.dutyCycleW;
        } break;
        case BldcFoc::BrushlessFocPwmControlType::SINE: {
            // Do an inverse clarke transform
            math::abc_pair_t inverse_clarke_transform =
                math::inverse_clarke_transform(inverse_park_transform.alpha, inverse_park_transform.beta);

            // load the results into the phase commands
            if (math::float_equals(bus_voltage, 0.0f)) {
                // Set the duty cycles to 0
                duty_cycle_u_h = 0.0f;
                duty_cycle_v_h = 0.0f;
                duty_cycle_w_h = 0.0f;
                break;
            }
            // The inverse clarke transforms produce a line-line voltage that is both positive
            // and negative. We can provide +- bus_voltage/2 to the high side and the low side
            // and should scale the duty cycle accordingly
            duty_cycle_u_h = inverse_clarke_transform.a / (bus_voltage / 2.0f);
            duty_cycle_v_h = inverse_clarke_transform.b / (bus_voltage / 2.0f);
            duty_cycle_w_h = inverse_clarke_transform.c / (bus_voltage / 2.0f);

            // Duty cycles can be between -1 and 1, and those should linearly map to 0 -> 1
            duty_cycle_u_h = (duty_cycle_u_h + max_duty_cycle) / (max_duty_cycle * 2.0f);
            duty_cycle_v_h = (duty_cycle_v_h + max_duty_cycle) / (max_duty_cycle * 2.0f);
            duty_cycle_w_h = (duty_cycle_w_h + max_duty_cycle) / (max_duty_cycle * 2.0f);

        } break;
        default:
            // Set the duty cycles to 0
            duty_cycle_u_h = 0.0f;
            duty_cycle_v_h = 0.0f;
            duty_cycle_w_h = 0.0f;
            break;
    }

    // No matter what, the duty cycles should be between 0 and 1
    math::clamp(duty_cycle_u_h, 0.0f, 1.0f);
    math::clamp(duty_cycle_v_h, 0.0f, 1.0f);
    math::clamp(duty_cycle_w_h, 0.0f, 1.0f);

    // Set the alpha and beta components of the voltage vector
    result.V_alpha_beta.alpha = inverse_park_transform.alpha;
    result.V_alpha_beta.beta = inverse_park_transform.beta;

    result.duty_cycle_u_h = duty_cycle_u_h;
    result.duty_cycle_v_h = duty_cycle_v_h;
    result.duty_cycle_w_h = duty_cycle_w_h;

    // Set the duty cycles
    phase_command_u.duty_cycle_high_side = duty_cycle_u_h;
    phase_command_u.invert_low_side = true;
    phase_command_v.duty_cycle_high_side = duty_cycle_v_h;
    phase_command_v.invert_low_side = true;
    phase_command_w.duty_cycle_high_side = duty_cycle_w_h;
    phase_command_w.invert_low_side = true;

    return result;
}

svpwm_duty_cycle svpwm(float Vd, float Vq, float theta_el, float Vbus) {
    svpwm_duty_cycle result;
    do {
        if (math::float_equals(Vbus, 0.0f)) {
            break;
        }
        // First, calcuate the magnitude of the voltage vector
        float Vmag = sqrtf(Vd * Vd + Vq * Vq);
        // Now, normalize the Vmag with the bus voltage
        float Vmag_norm = Vmag / Vbus;
        // Calculate the working theta that further adds the arctan of the voltage vector
        float svpwm_theta = theta_el + atan2f(Vq, Vd);
        // Wrap the working theta to be between 0 and 2pi
        math::wraparound(svpwm_theta, 0.0f, 2.0f * math::M_PI_FLOAT);
        // Calculate the sector
        // NOTE: This is a 1-indexed sector as 0 and 7 are reseved as null sectors
        uint8_t sector = static_cast<uint8_t>(svpwm_theta / (math::M_PI_FLOAT / 3.0f)) + 1;

        // Calculate the time T0,T1,T2 by using the normalized voltage magnitude
        // See https://www.youtube.com/watch?v=QMSWUMEAejg for the derivations of the equations below
        // Note: Tz (or total period) is always 1.0f
        // Vref/Vdc is accounted for with our Vmag_norm calculation above
        float T1 = math::sqrt_3 * (sector * math::M_PI_FLOAT / 3.0f - svpwm_theta) * Vmag_norm;
        float T2 = math::sqrt_3 * (svpwm_theta - (sector - 1) * math::M_PI_FLOAT / 3.0f) * Vmag_norm;
        float T0 = 1.0f - T1 - T2;

        // Now, calculate the duty cycles based on the sector
        switch (sector) {
            case 1:
                result.dutyCycleU = T1 + T2 + T0 / 2.0f;
                result.dutyCycleV = T2 + T0 / 2.0f;
                result.dutyCycleW = T0 / 2.0f;
                break;
            case 2:
                result.dutyCycleU = T1 + T0 / 2.0f;
                result.dutyCycleV = T1 + T2 + T0 / 2.0f;
                result.dutyCycleW = T0 / 2.0f;
                break;
            case 3:
                result.dutyCycleU = T0 / 2.0f;
                result.dutyCycleV = T1 + T2 + T0 / 2.0f;
                result.dutyCycleW = T2 + T0 / 2.0f;
                break;
            case 4:
                result.dutyCycleU = T0 / 2.0f;
                result.dutyCycleV = T1 + T0 / 2.0f;
                result.dutyCycleW = T1 + T2 + T0 / 2.0f;
                break;
            case 5:
                result.dutyCycleU = T2 + T0 / 2.0f;
                result.dutyCycleV = T0 / 2.0f;
                result.dutyCycleW = T1 + T2 + T0 / 2.0f;
                break;
            case 6:
                result.dutyCycleU = T1 + T2 + T0 / 2.0f;
                result.dutyCycleV = T0 / 2.0f;
                result.dutyCycleW = T1 + T0 / 2.0f;
                break;
            default:
                // wrong parta' down buddy (as an old boss would say)
                result.dutyCycleU = 0.0f;
                result.dutyCycleV = 0.0f;
                result.dutyCycleW = 0.0f;
                break;
        }
    } while (0);

    return result;
}

math::dq_pair_t convert_current_to_dq_frame(const hwbridge::Bridge3Phase::phase_current_t& phase_currents, float theta) {
    // Do a Clarke transform
    math::alpha_beta_pair_t clarke_transform = math::clarke_transform(phase_currents.u, phase_currents.v, phase_currents.w);

    // Do a Park transform
    math::dq_pair_t park_transform_currents = math::park_transform(clarke_transform.alpha, clarke_transform.beta, theta);

    return park_transform_currents;
}

math::dq_pair_t determine_voltage_vector_foc(pid::PID<float>& pi_id, pid::PID<float>& pi_iq, float i_q_reference,
                                             float i_d_reference, math::dq_pair_t i_dq, math::dq_pair_t V_dq_ff) {
    math::dq_pair_t result;
    // Run the PI controller
    // The below hack for speed is kinda hacky and should be reverted lol
    const float q_voltage_delta = pi_iq.calculate(i_dq.quadrature, i_q_reference);
    const float d_voltage_delta = pi_id.calculate(i_dq.direct, i_d_reference);
    result.quadrature = q_voltage_delta + V_dq_ff.quadrature;
    result.direct = d_voltage_delta + V_dq_ff.direct;

    return result;
}

math::dq_pair_t clamp_Vdq_vector(math::dq_pair_t V_dq, float bus_voltage) {
    math::dq_pair_t result;
    const float V_direct = V_dq.direct;
    const float V_quadrature = V_dq.quadrature;
    // Limit the Vd and Vq by first calculating the modulus of the vector
    const float V_modulus = sqrtf(V_direct * V_direct + V_quadrature * V_quadrature);
    const float max_Vmod = bus_voltage * 3.0f / 4.0f;
    // If the modulus is greater than the bus voltage, then we need to scale the voltage vector
    if (V_modulus > max_Vmod) {
        // Scale the voltage vector
        result.direct = V_direct * max_Vmod / V_modulus;
        result.quadrature = V_quadrature * max_Vmod / V_modulus;
    }
    return result;
}

}  // namespace BldcFoc
}  // namespace control_loop