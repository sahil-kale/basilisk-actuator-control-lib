#ifndef MATH_UTIL_HPP
#define MATH_UTIL_HPP
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace math {

constexpr float sqrt_3 = 1.7320508075688772f;
constexpr float sqrt_3_over_3 = 0.5773502691896258f;

constexpr float M_PI_FLOAT = static_cast<float>(M_PI);

constexpr float ACCEPTABLE_FLOAT_ERROR = 0.000001f;

// Make a floated function to determine whether two floats are equal
/**
 * @brief Determine whether two floats are equal.
 * @param a The first float
 * @param b The second float
 * @return bool Whether the floats are equal
 */
bool float_equals(float a, float b);

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

// Determine tau from f_c
/**
 * @brief Determine tau from f_c
 * @param f_c The cutoff frequency
 * @return float The time constant
 */
float determine_tau_from_f_c(float f_c);

/**
 * @brief Low pass filter the given input value.
 * @param input The input value
 * @param prev_output The previous output value
 * @param tau The time constant of the filter
 * @param dt The time step
 */
float low_pass_filter(float input, float prev_output, float tau, float dt);

// Make a function to get the sign of a number of any type
/**
 * @brief Return the sign of a number.
 * @param val The number to get the sign of
 * @return float The sign of the number
 */
template <typename T>
float sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// Return the maximum of two values
/**
 * @brief Return the maximum of two values.
 * @param a The first value
 * @param b The second value
 */
template <typename T>
T max(const T& a, const T& b) {
    return (a > b) ? a : b;
}

// Return the minimum of two values
/**
 * @brief Return the minimum of two values.
 * @param a The first value
 * @param b The second value
 */
template <typename T>
T min(const T& a, const T& b) {
    return (a < b) ? a : b;
}

}  // namespace math

#endif  // MATH_UTIL_HPP
