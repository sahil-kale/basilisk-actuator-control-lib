#ifndef MOCK_HAL_TIMER_HPP
#define MOCK_HAL_TIMER_HPP

#include "gmock/gmock.h"
#include "hal_timer.hpp"

namespace basilisk_hal {

class MOCK_HAL_TIMER : public HAL_Timer {
   public:
    MOCK_METHOD(void, set_timer_params, (timer_mode_E mode, uint32_t frequency), (override));
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(void, set_channel, (uint32_t ticks, timer_channel_E channel), (override));
    MOCK_METHOD(void, set_period, (uint32_t period), (override));
    MOCK_METHOD(void, reset, (), (override));
    MOCK_METHOD(uint32_t, get_period, (), (override));
};

// Create a complementary PWM timer class
class MOCK_HAL_ComplementaryPWM_Timer : public HAL_ComplementaryPWM_Timer {
   public:
    MOCK_METHOD(void, set_timer_params, (timer_mode_E mode, uint32_t frequency), (override));
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(void, set_channel, (uint32_t ticks, timer_channel_E channel), (override));
    MOCK_METHOD(void, set_period, (uint32_t period), (override));
    MOCK_METHOD(void, reset, (), (override));
    MOCK_METHOD(uint32_t, get_period, (), (override));
    MOCK_METHOD(void, set_channel_with_complementary_phase,
                (uint32_t ticks, timer_channel_E channel,
                 HAL_ComplementaryPWM_Timer::complementary_PWM_phase_E complementary_channel_phase),
                (override));
};

}  // namespace basilisk_hal

#endif