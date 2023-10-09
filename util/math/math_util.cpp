#include "math_util.hpp"

namespace math {
float low_pass_filter(float input, float prev_output, float tau, float dt) {
    float alpha = dt / (tau + dt);
    return alpha * input + (1.0f - alpha) * prev_output;
}

float trapezoidal_integral(float x, float x_prev, float y, float y_prev) {
    float integral = (x - x_prev) * (y + y_prev) / 2.0f;
    return integral;
}
}  // namespace math