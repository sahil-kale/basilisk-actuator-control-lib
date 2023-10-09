#ifndef HAL_TIMER_HPP
#define HAL_TIMER_HPP
#include <stdint.h>

namespace basilisk_hal {
// Create enum class for timer mode (PWM_EDGE_ALIGNED, PWM_CENTRE_ALIGNED)
enum class timer_mode_E {
    PWM_EDGE_ALIGNED,
    PWM_CENTRE_ALIGNED,
};

// Create enum class for timer channel up to 6
enum class timer_channel_E {
    TIMER_CHANNEL_1 = 1,
    TIMER_CHANNEL_2 = 2,
    TIMER_CHANNEL_3 = 3,
    TIMER_CHANNEL_4 = 4,
    TIMER_CHANNEL_5 = 5,
    TIMER_CHANNEL_6 = 6,
};

class HAL_Timer {
   public:
    HAL_Timer() = default;
    virtual ~HAL_Timer() {}

    // Function to set the parameters of the timer
    // TODO: Add more parameters into the function that check for timer channel etc etc
    virtual void set_timer_params(timer_mode_E mode, uint32_t frequency) = 0;

    // Function to start the timer
    virtual void start() = 0;

    // Function to stop the timer
    virtual void stop() = 0;

    // Function to reset the timer
    virtual void reset() = 0;

    // Function to set the timer ticks
    virtual void set_channel(uint32_t ticks, timer_channel_E channel) = 0;

    // Function to set the timer period
    virtual void set_period(uint32_t period) = 0;

    // Function to get the timer period
    virtual uint32_t get_period() = 0;

    // Function to start the pwm
    virtual void start_pwm(timer_channel_E channel) = 0;
};

// Create a complementary PWM timer class
class HAL_ComplementaryPWM_Timer : public HAL_Timer {
   public:
    HAL_ComplementaryPWM_Timer() = default;
    virtual ~HAL_ComplementaryPWM_Timer() {}

    // Declare an ENUM class specifying the complementary channel ACTIVE_LOW or ACTIVE_HIGH
    // Note: 'active' refers to when the main PWM is signal is asserted 'high' in most cases.
    // Thus, ACTIVE_LOW means that the complementary channel is asserted 'low' when the main PWM is asserted 'high'
    // while ACTIVE_HIGH means that the complementary channel is asserted 'high' when the main PWM is asserted 'high'
    enum class complementary_PWM_phase_E {
        ACTIVE_LOW,
        ACTIVE_HIGH,
    };

    // Allow the inverting and non-inverting channels to be used
    virtual void set_channel_with_complementary_phase(uint32_t ticks, timer_channel_E channel,
                                                      complementary_PWM_phase_E complementary_channel_phase) = 0;
};

}  // namespace basilisk_hal

#endif
