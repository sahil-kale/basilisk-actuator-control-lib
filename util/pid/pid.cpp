#include "pid.hpp"

#include "math_util.hpp"

namespace pid {

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
    if (integral_windup != 0) {
        math::clamp(integral, -integral_windup, integral_windup);
    }

    float derivative = 0;

    // Calculate the derivative term
    if (dt != 0) {
        derivative = (error - previous_error) / dt;
    }
    // Store the error for the next time
    previous_error = error;
    last_time = current_time;

    T output = error * kp + integral * ki + derivative * kd;

    // Clamp the output
    if ((max_output != 0) || (min_output != 0)) {
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

}  // namespace pid