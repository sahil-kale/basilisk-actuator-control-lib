#ifndef PID_HPP
#define PID_HPP
#include "hal_clock.hpp"

namespace pid {

// Create a PID controller that takes in a setpoint and a feedback value and returns a control value.
// The class is templated so that it can be used with any type that supports the basic arithmetic operators.
// The class should also take in initial P, I, and D gains as well as a maximum and minimum output value.
// Integral windup should be taken as a construction parameter

template <typename T>
class PID {
   public:
    /**
     * @brief Construct a new PID object
     * @param kp The proportional gain
     * @param ki The integral gain
     * @param kd The derivative gain
     * @param min_output The maximum output value
     * @param max_output The minimum output value
     * @param integral_windup The integral windup value
     */
    PID(float kp, float ki, float kd, T min_output, T max_output, float integral_windup, basilisk_hal::HAL_CLOCK* clock = nullptr)
        : kp(kp), ki(ki), kd(kd), max_output(max_output), min_output(min_output), integral_windup(integral_windup), clock(clock) {
        reset();
    }

    // Reset the PID controller
    void reset();

    // Register a clock object to be used for timing
    void register_clock(basilisk_hal::HAL_CLOCK* clock);

    // PID getter functions (marked as const)
    float get_kp() const;
    float get_ki() const;
    float get_kd() const;

    // PID setter functions
    void set_kp(float kp);
    void set_ki(float ki);
    void set_kd(float kd);

    // Set the maximum and minimum output values
    void set_max_output(T max_output);
    void set_min_output(T min_output);

    // Set the integral windup value
    void set_integral_windup(float integral_windup);

    // Calculate the control value given the setpoint and feedback value
    /**
     * @brief Calculate the control value given the setpoint and feedback value
     * @param setpoint The setpoint value
     * @param actual The actual value
     */
    T calculate(T actual, T setpoint);

   private:
    float kp;
    float ki;
    float kd;
    T max_output;
    T min_output;
    float integral_windup;
    // Integral and previous error term storage
    float integral;
    T previous_error;

    // Clock object to be used for timing
    basilisk_hal::HAL_CLOCK* clock = nullptr;
    utime_t last_time = 0;
};

}  // namespace pid

#endif