#include "math_util.hpp"

#include <cmath>

namespace math {
float low_pass_filter(float input, float prev_output, float tau, float dt) {
    float alpha = dt / (tau + dt);
    return alpha * input + (1.0f - alpha) * prev_output;
}

float determine_tau_from_f_c(float f_c) { return 1.0f / (2.0f * M_PI_FLOAT * f_c); }

float trapezoidal_integral(float x, float x_prev, float y, float y_prev) {
    float integral = (x - x_prev) * (y + y_prev) / 2.0f;
    return integral;
}

bool float_equals(float a, float b) { return fabs(a - b) < ACCEPTABLE_FLOAT_ERROR; }
}  // namespace math