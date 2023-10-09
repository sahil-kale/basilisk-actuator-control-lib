#ifndef INPUT_MANAGER_HPP
#define INPUT_MANAGER_HPP
#include "hal_adc.hpp"
#include "hal_common.hpp"

namespace input_manager {

class InputManager {
   public:
    static InputManager& getInstance() {
        static InputManager instance;
        return instance;
    }

    /**
     * @brief Register the HAL_ADC object with the input manager
     * @param hal_adc The HAL_ADC object to register
     * @return None
     */
    void register_hal_adc(basilisk_hal::HAL_ADC* hal_adc) { this->hal_adc = hal_adc; }

    /**
     * @brief Get the board temperature
     * @param temperature The temperature of the PCBA from surface mount sensor
     * @return The app hal status
     */
    app_hal_status_E get_board_temperature(float& temperature);

    /**
     * @brief Get the bus voltage
     * @param voltage The voltage of the bus
     * @return The app hal status
     */
    app_hal_status_E get_bus_voltage(float& voltage);

    /**
     * @brief Get the current
     * @param current The current going through the motor at time of poll
     * @return The app hal status
     */
    app_hal_status_E get_motor_current(float& current);

   private:
    basilisk_hal::HAL_ADC* hal_adc = nullptr;
    InputManager();
};

}  // namespace input_manager

#endif  // INPUT_MANAGER_HPP