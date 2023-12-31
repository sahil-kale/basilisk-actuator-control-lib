#ifndef HAL_CLOCK_HPP
#define HAL_CLOCK_HPP
#include <stdint.h>

#include "hal_common.hpp"

namespace basilisk_hal {

/**
 * @brief Abstract class for a clock that keeps track of the time since boot
 * @note This class should provide a way to get the current time in microseconds
 */
class HAL_CLOCK {
   public:
    HAL_CLOCK() = default;
    virtual ~HAL_CLOCK() {}
    /**
     * @brief Get the current time in microseconds
     * @return The current time in microseconds
     */
    virtual utime_t get_time_us() = 0;

    /**
     * @brief Get the current time in seconds
     * @return The current time in seconds
     */
    float get_time_s() { return get_time_us() / kMicrosecondsPerSecond; }

    /**
     * @brief Get the time difference between two times in microseconds
     * @param t1 The first time
     * @param t2 The second time
     * @return The time difference in microseconds
     */
    static utime_t get_dt_us(utime_t t1, utime_t t2) { return t1 - t2; }

    /**
     * @brief Get the time difference between two times in microseconds
     * @param t1 The first time
     * @param t2 The second time
     * @return The time difference in microseconds
     */
    static float get_dt_s(utime_t t1, utime_t t2) { return static_cast<float>(get_dt_us(t1, t2)) / kMicrosecondsPerSecond; }

    /**
     * @brief Number of microseconds per second
     */
    static constexpr float kMicrosecondsPerSecond = 1000000.0f;
};

}  // namespace basilisk_hal

#endif  // HAL_CLOCK_HPP