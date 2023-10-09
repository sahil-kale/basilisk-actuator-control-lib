#ifndef HAL_GPIO_HPP
#define HAL_GPIO_HPP

#include <stdint.h>

#include "hal_common.hpp"

namespace basilisk_hal {
// Create enum class for GPIO mode (INPUT, OUTPUT, ANALOG, ALTERNATE)
enum class gpio_mode_E {
    INPUT,
    OUTPUT,
    ANALOG,
    ALTERNATE,
};

class HAL_GPIO {
   public:
    HAL_GPIO() = default;
    virtual ~HAL_GPIO() {}

    /**
     * @brief Set the mode of a given GPIO pin
     * @param mode The mode to set the pin to
     * @return The status of the operation
     */
    virtual app_hal_status_E set_mode(gpio_mode_E mode) = 0;

    /**
     * @brief Set the output state of a given GPIO pin
     * @param pin The pin to set the output state of
     * @param state The state to set the pin to
     * @return The status of the operation
     */
    virtual app_hal_status_E set_output_state(bool state) = 0;

    /**
     * @brief Get the input state of a given GPIO pin
     * @param pin The pin to get the input state of
     * @param state The state to get the pin to
     * @return The status of the operation
     */
    virtual app_hal_status_E get_input_state(bool& state) = 0;

   protected:
    gpio_mode_E mode;
};
}  // namespace basilisk_hal

#endif