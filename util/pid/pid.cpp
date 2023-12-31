#include "pid.hpp"

#include "math_util.hpp"

namespace pid {
// Disable -Wconversion for this file because of the template type 'int' being used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"

// PID getter functions (marked as const)
template <typename T>
float PID<T>::get_kp() const {
    return kp;
}

template <typename T>
float PID<T>::get_ki() const {
    return ki;
}

template <typename T>
float PID<T>::get_kd() const {
    return kd;
}

// PID setter functions
template <typename T>
void PID<T>::set_kp(float kp) {
    this->kp = kp;
}
template <typename T>
void PID<T>::set_ki(float ki) {
    this->ki = ki;
}
template <typename T>
void PID<T>::set_kd(float kd) {
    this->kd = kd;
}

template <typename T>
T PID<T>::calculate(T actual, T setpoint) {
    // Calculate the error
    T error = setpoint - actual;
    const utime_t current_time = clock.get_time_us();
    float dt = clock.get_dt_s(current_time, last_time);
    if (use_dt == false) {
        dt = 1;
    }

    // Calculate the integral term
    integral += error * dt;
    // Clamp the integral term
    if (math::float_equals(integral_windup, 0) == false) {
        math::clamp(integral, -integral_windup, integral_windup);
    }

    float derivative = 0;

    // Calculate the derivative term
    if (math::float_equals(dt, 0) == false) {
        derivative = (error - previous_error) / dt;
    }
    // Store the error for the next time
    previous_error = error;
    last_time = current_time;

    T output = error * kp + integral * ki + derivative * kd;

    // Clamp the output
    const bool max_output_set = math::float_equals(max_output, 0) == false;
    const bool min_output_set = math::float_equals(min_output, 0) == false;
    if ((max_output_set) || (min_output_set)) {
        math::clamp(output, min_output, max_output);
    }
    return output;
}

// Reset the PID controller
template <typename T>
void PID<T>::reset() {
    integral = 0;
    previous_error = 0;
    last_time = clock.get_time_us();
}

// Set the maximum and minimum output values
template <typename T>
void PID<T>::set_max_output(T max_output) {
    this->max_output = max_output;
}

template <typename T>
void PID<T>::set_min_output(T min_output) {
    this->min_output = min_output;
}

// Set the integral windup value
template <typename T>
void PID<T>::set_integral_windup(float integral_windup) {
    this->integral_windup = integral_windup;
}

template class PID<float>;
template class PID<int>;

#pragma GCC diagnostic pop

}  // namespace pid