#include "foc_util.hpp"

#include <cmath>

#include "math_foc.hpp"
#include "math_util.hpp"
#include "util.hpp"

namespace control_loop {
namespace BldcFoc {

FocDutyCycleResult determine_inverter_duty_cycles_foc(float theta, float Vdirect, float Vquadrature, float bus_voltage,
                                                      BrushlessFocPwmControlType pwm_control_type,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_u,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_v,
                                                      hwbridge::Bridge3Phase::phase_command_t& phase_command_w) {
    const float max_duty_cycle = 1.0f;
    // Create the result
    FocDutyCycleResult result;
    // Do an inverse Park transform
    math::alpha_beta_t inverse_park_transform = math::inverse_park_transform(Vdirect, Vquadrature, theta);
    float duty_cycle_u_h = 0.0f;
    float duty_cycle_v_h = 0.0f;
    float duty_cycle_w_h = 0.0f;

    switch (pwm_control_type) {
        case BldcFoc::BrushlessFocPwmControlType::SPACE_VECTOR: {
            math::abc_t duty_cycles = svpwm(inverse_park_transform, bus_voltage);
            duty_cycle_u_h = duty_cycles.a;
            duty_cycle_v_h = duty_cycles.b;
            duty_cycle_w_h = duty_cycles.c;
        } break;
        case BldcFoc::BrushlessFocPwmControlType::SINE: {
            math::abc_t duty_cycles = sine_pwm(inverse_park_transform, bus_voltage, max_duty_cycle);
            duty_cycle_u_h = duty_cycles.a;
            duty_cycle_v_h = duty_cycles.b;
            duty_cycle_w_h = duty_cycles.c;

        } break;
        default:
            // Set the duty cycles to 0
            duty_cycle_u_h = 0.0f;
            duty_cycle_v_h = 0.0f;
            duty_cycle_w_h = 0.0f;
            break;
    }

    // No matter what, the duty cycles should be between 0 and 1
    math::clamp(duty_cycle_u_h, 0.0f, max_duty_cycle);
    math::clamp(duty_cycle_v_h, 0.0f, max_duty_cycle);
    math::clamp(duty_cycle_w_h, 0.0f, max_duty_cycle);

    // Set the alpha and beta components of the voltage vector
    result.V_alpha_beta = inverse_park_transform;

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

math::abc_t sine_pwm(math::alpha_beta_t V_alpha_beta, float Vbus, float max_duty_cycle) {
    math::abc_t result;
    do {
        math::abc_t inverse_clarke_transform = math::inverse_clarke_transform(V_alpha_beta.alpha, V_alpha_beta.beta);

        // load the results into the phase commands
        if (math::float_equals(Vbus, 0.0f)) {
            break;
        }
        // The inverse clarke transforms produce a line-line voltage that is both positive
        // and negative. We can provide +- bus_voltage/2 to the high side and the low side
        // and should scale the duty cycle accordingly
        const float max_phase_to_neutral_voltage = (Vbus / 2.0f);
        result.a = inverse_clarke_transform.a / max_phase_to_neutral_voltage;
        result.b = inverse_clarke_transform.b / max_phase_to_neutral_voltage;
        result.c = inverse_clarke_transform.c / max_phase_to_neutral_voltage;

        // Duty cycles can be between -1 and 1, and those should linearly map to 0 -> 1
        result.a = (result.a + max_duty_cycle) / (max_duty_cycle * 2.0f);
        result.b = (result.b + max_duty_cycle) / (max_duty_cycle * 2.0f);
        result.c = (result.c + max_duty_cycle) / (max_duty_cycle * 2.0f);

    } while (false);
    return result;
}

math::abc_t svpwm(math::alpha_beta_t V_alpha_beta, float Vbus) {
    math::abc_t result;
    do {
        // If the bus voltage is 0, then we can't do anything
        if (math::float_equals(Vbus, 0.0f)) {
            break;
        }

        // Determine the magnitude of the voltage vector
        const float V_modulus = sqrtf(V_alpha_beta.alpha * V_alpha_beta.alpha + V_alpha_beta.beta * V_alpha_beta.beta);

        // Determine alpha, the angle of the voltage vector
        float alpha = atan2f(V_alpha_beta.beta, V_alpha_beta.alpha);

        // atan2f returns a value between -pi and pi, but we want a value between 0 and 2pi
        if (alpha < 0.0f) {
            alpha += 2.0f * M_PI;
        }

        // Normalize our calculations to 1.0 time period, such that our switching times can be converted directly to duty cycles
        constexpr float T_z = 1.0f;

        const uint8_t sector = svm_sector(V_alpha_beta);

        uint8_t sector_n_minus_1 = sector;
        // The sector convention 'n' wraps around to 6
        if (sector_n_minus_1 == 1) {
            sector_n_minus_1 = 6;
        } else {
            sector_n_minus_1--;
        }

        // This is the term that is common to T1 and T2 for multiplication
        const float timing_multiplier_const = math::sqrt_3 * T_z * V_modulus / Vbus;
        const float T1 = timing_multiplier_const * (sinf(sector / 3.0f * M_PI - alpha));
        const float T2 = timing_multiplier_const * (sinf(alpha - (sector_n_minus_1) / 3.0f * M_PI));
        const float T0 = T_z - T1 - T2;

        switch (sector) {
            case 1:
                result.a = T1 + T2 + T0 / 2.0f;
                result.b = T2 + T0 / 2.0f;
                result.c = T0 / 2.0f;
                break;
            case 2:
                result.a = T1 + T0 / 2.0f;
                result.b = T1 + T2 + T0 / 2.0f;
                result.c = T0 / 2.0f;
                break;
            case 3:
                result.a = T0 / 2.0f;
                result.b = T1 + T2 + T0 / 2.0f;
                result.c = T2 + T0 / 2.0f;
                break;
            case 4:
                result.a = T0 / 2.0f;
                result.b = T1 + T0 / 2.0f;
                result.c = T1 + T2 + T0 / 2.0f;
                break;
            case 5:
                result.a = T2 + T0 / 2.0f;
                result.b = T0 / 2.0f;
                result.c = T1 + T2 + T0 / 2.0f;
                break;
            case 6:
                result.a = T1 + T2 + T0 / 2.0f;
                result.b = T0 / 2.0f;
                result.c = T1 + T0 / 2.0f;
                break;

            default:
                // something went wrong...
                result = math::abc_t();
                break;
        }

    } while (false);

    return result;
}

uint8_t svm_sector(math::alpha_beta_t V_alpha_beta) {
    uint8_t sector = 0;
    // Get the angle of the voltage vector
    const float theta = atan2f(V_alpha_beta.beta, V_alpha_beta.alpha);
    do {
        if (theta >= 0.0f && theta < M_PI / 3.0f) {
            sector = 1;
            break;
        }
        if (theta >= M_PI / 3.0f && theta < 2.0f * M_PI / 3.0f) {
            sector = 2;
            break;
        }
        if (theta >= 2.0f * M_PI / 3.0f && theta < M_PI) {
            sector = 3;
            break;
        }
        if (theta >= -M_PI && theta < -2.0f * M_PI / 3.0f) {
            sector = 4;
            break;
        }
        if (theta >= -2.0f * M_PI / 3.0f && theta < -M_PI / 3.0f) {
            sector = 5;
            break;
        }
        if (theta >= -M_PI / 3.0f && theta < 0.0f) {
            sector = 6;
            break;
        }
    } while (false);
    return sector;
}

math::direct_quad_t clamp_Vdq(math::direct_quad_t V_dq, float V_bus) {
    // Limit the Vd and Vq by first calculating the modulus of the vector
    const float V_modulus = sqrtf(V_dq.direct * V_dq.direct + V_dq.quadrature * V_dq.quadrature);
    const float max_Vmod = V_bus * 3.0f / 4.0f;
    // If the modulus is greater than the bus voltage, then we need to scale the voltage vector
    if (V_modulus > max_Vmod) {
        // Scale the voltage vector
        V_dq.direct = V_dq.direct * max_Vmod / V_modulus;
        V_dq.quadrature = V_dq.quadrature * max_Vmod / V_modulus;
    }

    return V_dq;
}

}  // namespace BldcFoc
}  // namespace control_loop