#ifndef PID_HPP
#define PID_HPP
#include "hal_clock.hpp"

namespace pid {

/**
 * @brief discrete-time PID controller class
 */
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
     * @param clock The clock object to be used for timing
     * @param use_dt Flag to use dt in the integral and derivative terms (this makes the PID controller invariant to the sampling
     * time by using dt in the integral and derivative terms)
     */
    PID(float kp, float ki, float kd, T min_output, T max_output, float integral_windup, basilisk_hal::HAL_CLOCK& clock,
        bool use_dt = true)
        : kp(kp),
          ki(ki),
          kd(kd),
          max_output(max_output),
          min_output(min_output),
          integral_windup(integral_windup),
          clock(clock),
          use_dt(use_dt) {}

    /**
     * @brief reset the PID controller
     */
    void reset();

    /**
     * @brief Get the kp value
     * @return The kp value
     */

    float get_kp() const;
    /**
     * @brief Get the ki value
     * @return The ki value
     */
    float get_ki() const;

    /**
     * @brief Get the kd value
     * @return The kd value
     */
    float get_kd() const;

    /**
     * @brief set the kp value
     * @param kp The kp value
     */
    void set_kp(float kp);

    /**
     * @brief set the ki value
     * @param ki The ki value
     */
    void set_ki(float ki);

    /**
     * @brief set the kd value
     * @param kd The kd value
     */
    void set_kd(float kd);

    /**
     * @brief Set the max output value
     * @param max_output The max output value
     */
    void set_max_output(T max_output);

    /**
     * @brief Set the min output value
     * @param min_output The min output value
     */
    void set_min_output(T min_output);

    /**
     * @brief Set the integral windup value
     * @param integral_windup The integral windup value as an absolute value
     */
    void set_integral_windup(float integral_windup);

    // Calculate the control value given the setpoint and feedback value
    /**
     * @brief Calculate the control value given the setpoint and feedback value
     * @param actual The actual value
     * @param setpoint The setpoint value
     * @return The control value
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
    float integral{0};
    T previous_error{0};

    // Clock object to be used for timing
    basilisk_hal::HAL_CLOCK& clock;
    utime_t last_time = 0;

    // Option flagging
    bool use_dt;
};

}  // namespace pid

#endif