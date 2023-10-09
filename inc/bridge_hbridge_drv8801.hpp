#ifndef BRIDGE_HBRIDGE_DRV8801_HPP
#define BRIDGE_HBRIDGE_DRV8801_HPP
#include <math.h>

#include "bridge_hbridge.hpp"
#include "control_loop.hpp"
#include "hal_gpio.hpp"
#include "hal_timer.hpp"

namespace hwbridge {

// Define a class that inherits from HBridge
class HBridgeDRV8801 : public HBridge {
   public:
    HBridgeDRV8801() = default;
    ~HBridgeDRV8801() = default;

    void init(basilisk_hal::HAL_Timer& timer, basilisk_hal::timer_channel_E channel, basilisk_hal::HAL_GPIO& pin,
              uint32_t frequency) {
        // Register the timer and pin
        pwm_timer_ = &timer;
        dir_pin_ = &pin;
        channel_ = channel;

        // Set the direction pin to output
        dir_pin_->set_mode(basilisk_hal::gpio_mode_E::OUTPUT);

        // Set the pwm timer to PWM_CENTRE_ALIGNED
        pwm_timer_->set_timer_params(basilisk_hal::timer_mode_E::PWM_CENTRE_ALIGNED, frequency);

        // Set the timer channel
        pwm_timer_->set_channel(0, channel_);

        // Start the timer
        pwm_timer_->start();
    }

    // Define a run method that overrides the run method in HBridge
    void run(float speed) override {
        // Set the direction pin
        dir_pin_->set_output_state(speed > 0);

        // Determine the appropriate amount of ticks by getting the timer period
        uint32_t period = pwm_timer_->get_period();
        uint32_t ticks = static_cast<uint32_t>(period * fabs(speed) / control_loop::ControlLoop::MAX_MOTOR_SPEED);

        // Set the PWM timer
        pwm_timer_->set_channel(ticks, channel_);
    }

   private:
    // Define a timer object
    basilisk_hal::HAL_Timer* pwm_timer_;
    basilisk_hal::timer_channel_E channel_;
    basilisk_hal::HAL_GPIO* dir_pin_;
};

}  // namespace hwbridge
#endif  // BRIDGE_HBRIDGE_DRV8801_HPP