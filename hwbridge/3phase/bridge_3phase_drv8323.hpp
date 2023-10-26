#ifndef BRIDGE_3PHASE_DRV8323_HPP
#define BRIDGE_3PHASE_DRV8323_HPP

#include <math.h>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "hal_adc.hpp"
#include "hal_common.hpp"
#include "hal_gpio.hpp"
#include "hal_timer.hpp"
#include "util.hpp"

namespace hwbridge {

// Define a 3-phase bridge class using DRV8323
class Bridge3PhaseDRV8323 : public Bridge3Phase {
   public:
    typedef struct drv8323_phase_config_info {
        // Timer channel
        // TODO: there should be a timer object here as well. Assuming all channels are on the same timer
        basilisk_hal::timer_channel_E channel;
        basilisk_hal::HAL_GPIO* low_side_gpio;  // Not supporting low side gate control for now except for enable/disable
        basilisk_hal::HAL_ADC* bemf_sense_adc;
        uint8_t bemf_sense_adc_channel;
        // TODO: add current sense support. For now, only 6 step is supported.
        basilisk_hal::HAL_ADC* current_sense_adc;
        uint8_t current_sense_adc_channel;
    } drv8323_phase_config_info_t;

    Bridge3PhaseDRV8323(basilisk_hal::HAL_ComplementaryPWM_Timer& timer, drv8323_phase_config_info_t& u,
                        drv8323_phase_config_info_t& v, drv8323_phase_config_info_t& w, uint32_t frequency)
        : pwm_timer_(&timer), u_(u), v_(v), w_(w), frequency_(frequency){};
    ~Bridge3PhaseDRV8323() = default;

    app_hal_status_E init() override {
        // Set the direction pin to output
        u_.low_side_gpio->set_mode(basilisk_hal::gpio_mode_E::OUTPUT);
        v_.low_side_gpio->set_mode(basilisk_hal::gpio_mode_E::OUTPUT);
        w_.low_side_gpio->set_mode(basilisk_hal::gpio_mode_E::OUTPUT);

        // Set the pwm timer to PWM_CENTRE_ALIGNED
        pwm_timer_->set_timer_params(basilisk_hal::timer_mode_E::PWM_CENTRE_ALIGNED, frequency_);

        // Set the timer channel
        pwm_timer_->set_channel(0, u_.channel);
        pwm_timer_->set_channel(0, v_.channel);
        pwm_timer_->set_channel(0, w_.channel);

        // Start the timer
        pwm_timer_->start();

        // Start PWM
        pwm_timer_->start_pwm(u_.channel);
        pwm_timer_->start_pwm(v_.channel);
        pwm_timer_->start_pwm(w_.channel);

        return app_hal_status_E::APP_HAL_OK;
    }

    // Define a virtual function to set the individual phases' duty cycles and enable/disable the phase
    app_hal_status_E set_phase(const phase_command_t& u, const phase_command_t& v, const phase_command_t& w) {
        // Determine the appropriate amount of ticks by getting the timer period
        uint32_t period = pwm_timer_->get_period();
        uint32_t ticks_u =
            static_cast<uint32_t>(period * fabs(u.duty_cycle_high_side) / control_loop::ControlLoop::MAX_MOTOR_SPEED);
        uint32_t ticks_v =
            static_cast<uint32_t>(period * fabs(v.duty_cycle_high_side) / control_loop::ControlLoop::MAX_MOTOR_SPEED);
        uint32_t ticks_w =
            static_cast<uint32_t>(period * fabs(w.duty_cycle_high_side) / control_loop::ControlLoop::MAX_MOTOR_SPEED);

        // Determine the low-side phase active high or active low signal for the complementary timer
        // If the inverted boolean is set in the phase command, we should set the command
        // to active low, otherwise, we should set the command to active high

        basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E u_complementary_phase =
            u.invert_low_side ? basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_LOW
                              : basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_HIGH;

        basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E v_complementary_phase =
            v.invert_low_side ? basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_LOW
                              : basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_HIGH;

        basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E w_complementary_phase =
            w.invert_low_side ? basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_LOW
                              : basilisk_hal::HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E::ACTIVE_HIGH;

        // Set the PWM timer
        pwm_timer_->set_channel_with_complementary_phase(ticks_u, u_.channel, u_complementary_phase);
        pwm_timer_->set_channel_with_complementary_phase(ticks_v, v_.channel, v_complementary_phase);
        pwm_timer_->set_channel_with_complementary_phase(ticks_w, w_.channel, w_complementary_phase);

        return app_hal_status_E::APP_HAL_OK;
    }

    // Define a function to read the BEMF voltage
    app_hal_status_E read_bemf(phase_voltage_t& bemf_voltage) {
        app_hal_status_E status = app_hal_status_E::APP_HAL_OK;
        if (u_.bemf_sense_adc == nullptr || v_.bemf_sense_adc == nullptr || w_.bemf_sense_adc == nullptr) {
            status = app_hal_status_E::APP_HAL_NOT_INITIALIZED;
        } else {
            do {
                status = u_.bemf_sense_adc->read_adc(bemf_voltage.u, u_.bemf_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
                status = v_.bemf_sense_adc->read_adc(bemf_voltage.v, v_.bemf_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
                status = w_.bemf_sense_adc->read_adc(bemf_voltage.w, w_.bemf_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
            } while (0);
        }

        return status;
    }

    // Define a function to read the current
    app_hal_status_E read_current(phase_current_t& current) {
        app_hal_status_E status = app_hal_status_E::APP_HAL_OK;
        if (u_.current_sense_adc == nullptr || v_.current_sense_adc == nullptr || w_.current_sense_adc == nullptr) {
            status = app_hal_status_E::APP_HAL_NOT_INITIALIZED;
        } else {
            do {
                status = u_.current_sense_adc->read_adc(current.u, u_.current_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
                status = v_.current_sense_adc->read_adc(current.v, v_.current_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
                status = w_.current_sense_adc->read_adc(current.w, w_.current_sense_adc_channel);
                if (status != app_hal_status_E::APP_HAL_OK) {
                    break;
                }
            } while (0);
        }

        return status;
    }

   private:
    // Define a timer object
    basilisk_hal::HAL_ComplementaryPWM_Timer* pwm_timer_;
    drv8323_phase_config_info_t u_;
    drv8323_phase_config_info_t v_;
    drv8323_phase_config_info_t w_;
    uint32_t frequency_;
    static constexpr uint8_t BEMF_VOLTAGE_AVERAGE_SIZE = 10;
    // Define an array of bemf voltage objects to average out the readings
    phase_voltage_t bemf_voltage_[BEMF_VOLTAGE_AVERAGE_SIZE];
};

}  // namespace hwbridge

#endif  // BRIDGE_3PHASE_DRV8323_HPP
