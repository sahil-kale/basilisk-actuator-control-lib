#include "math_foc.hpp"

#include "math.h"
#include "math_util.hpp"

namespace math {

clarke_transform_result_t clarke_transform(float a, float b, float c) {
    clarke_transform_result_t result;
    result.alpha = a;
    result.beta = (b - c) / sqrt_3;
    return result;
}

park_transform_result_t park_transform(float alpha, float beta, float theta) {
    park_transform_result_t result;
    float theta_rad = theta;
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    result.d = alpha * cos_theta + beta * sin_theta;
    result.q = -alpha * sin_theta + beta * cos_theta;
    return result;
}

inverse_park_transform_result_t inverse_park_transform(float d, float q, float theta) {
    inverse_park_transform_result_t result;
    float theta_rad = theta;
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    result.alpha = d * cos_theta - q * sin_theta;
    result.beta = d * sin_theta + q * cos_theta;
    return result;
}

inverse_clarke_transform_result_t inverse_clarke_transform(float alpha, float beta) {
    inverse_clarke_transform_result_t result;
    result.a = alpha;
    result.b = (-alpha + sqrt_3 * beta) / 2.0f;
    result.c = (-alpha - sqrt_3 * beta) / 2.0f;
    return result;
}

svpwm_duty_cycle_t svpwm(float Vd, float Vq, float theta_el, float Vbus) {
    svpwm_duty_cycle_t result = {0.0f, 0.0f, 0.0f};
    // First, calcuate the magnitude of the voltage vector
    float Vmag = sqrtf(Vd * Vd + Vq * Vq);
    // Now, normalize the Vmag with the bus voltage
    float Vmag_norm = Vmag / Vbus;
    // Calculate the working theta that further adds the arctan of the voltage vector
    float svpwm_theta = theta_el + atan2f(Vq, Vd);
    // Wrap the working theta to be between 0 and 2pi
    math::wraparound(svpwm_theta, 0.0f, 2.0f * M_PI_FLOAT);
    // Calculate the sector
    // NOTE: This is a 1-indexed sector as 0 and 7 are reseved as null sectors
    uint8_t sector = static_cast<uint8_t>(svpwm_theta / (M_PI_FLOAT / 3.0f)) + 1;

    // Calculate the time T0,T1,T2 by using the normalized voltage magnitude
    // See https://www.youtube.com/watch?v=QMSWUMEAejg for the derivations of the equations below
    // Note: Tz (or total period) is always 1.0f
    // Vref/Vdc is accounted for with our Vmag_norm calculation above
    float T1 = sqrt_3 * (sector * M_PI_FLOAT / 3.0f - svpwm_theta) * Vmag_norm;
    float T2 = sqrt_3 * (svpwm_theta - (sector - 1) * M_PI_FLOAT / 3.0f) * Vmag_norm;
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

    return result;
}

}  // namespace math