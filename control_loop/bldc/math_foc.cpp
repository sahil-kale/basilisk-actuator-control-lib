#include "math_foc.hpp"

#include "math.h"
#include "math_util.hpp"

namespace math {

clarke_transform_result_t clarke_transform(float a, float b, float c) {
    // Note: uses simplification that ia + ib + ic = 0 to derive the simplified version
    clarke_transform_result_t result;
    result.alpha = a;
    result.beta = (b - c) / sqrt_3;
    return result;
}

park_transform_result_t park_transform(float alpha, float beta, float theta) {
    park_transform_result_t result;
    const float theta_rad = theta;
    const float cos_theta = cos(theta_rad);
    const float sin_theta = sin(theta_rad);
    result.d = alpha * cos_theta + beta * sin_theta;
    result.q = -alpha * sin_theta + beta * cos_theta;
    return result;
}

inverse_park_transform_result_t inverse_park_transform(float d, float q, float theta) {
    inverse_park_transform_result_t result;
    const float theta_rad = theta;
    const float cos_theta = cos(theta_rad);
    const float sin_theta = sin(theta_rad);
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

}  // namespace math