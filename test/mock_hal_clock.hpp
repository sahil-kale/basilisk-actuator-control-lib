#ifndef MOCK_HAL_CLOCK_HPP
#define MOCK_HAL_CLOCK_HPP

#include "gmock/gmock.h"
#include "hal_clock.hpp"

namespace basilisk_hal {

class MOCK_HAL_CLOCK : public HAL_CLOCK {
   public:
    MOCK_METHOD(utime_t, get_time_us, (), (override));
};

}  // namespace basilisk_hal
#endif