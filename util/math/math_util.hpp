#ifndef MATH_UTIL_HPP
#define MATH_UTIL_HPP

namespace math {
/**
 * @brief Integrate the given value using the trapezoidal rule.
 * @param x The current x value
 * @param x_prev The previous x value
 * @param y The current y value
 * @param y_prev The previous y value
 */
float trapezoidal_integral(float x, float x_prev, float y, float y_prev);

/**
 * @brief Clamp the given value between the given min and max values.
 * @param value The value to clamp
 * @param min The minimum value
 * @param max The maximum value
 * @return void
 */
template <typename T>
void clamp(T& value, const T& min, const T& max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    } else {
        // do nothing
    }
}

/**
 * @brief Wrap the given value around the given min and max values.
 * @param value The value to wrap
 * @param min The minimum value
 * @param max The maximum value
 * @return void
 */
template <typename T>
void wraparound(T& value, const T& min, const T& max) {
    while (value < min) {
        value += (max - min);
    }
    while (value > max) {
        value -= (max - min);
    }
}

/**
 * @brief Low pass filter the given input value.
 * @param input The input value
 * @param prev_output The previous output value
 * @param tau The time constant of the filter
 * @param dt The time step
 */
float low_pass_filter(float input, float prev_output, float tau, float dt);
}  // namespace math

#endif  // MATH_UTIL_HPP
