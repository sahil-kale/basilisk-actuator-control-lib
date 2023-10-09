#ifndef HAL_CLOCK_HPP
#define HAL_CLOCK_HPP
#include <stdint.h>

#include "hal_common.hpp"

namespace basilisk_hal {

// Create a clock class that can be used to get the current time in microseconds via singleton pattern
class HAL_CLOCK {
   public:
    HAL_CLOCK() = default;
    virtual ~HAL_CLOCK() {}
    /**
     * @brief Get the current time in microseconds
     * @return The current time in microseconds
     */
    virtual utime_t get_time_us() = 0;

    // Add a constant for the number of microseconds in a second
    static constexpr float kMicrosecondsPerSecond = 1000000.0f;
};

}  // namespace basilisk_hal

#endif  // HAL_CLOCK_HPP