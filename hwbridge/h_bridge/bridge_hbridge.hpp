#ifndef BRIDGE_HBRIDGE_HPP
#define BRIDGE_HBRIDGE_HPP
#include "hal_common.hpp"
#include "math.h"
#include "math_util.hpp"

namespace hwbridge {

/**
 * @brief Abstract class for an H bridge that can be used to control a motor
 * @note This class assumes the A -> B side is a positive voltage across the H bridge
 * @note See the `HBridgeInput` class for more information on the input to the H bridge
 */
class HBridge {
   public:
    HBridge() = default;
    virtual ~HBridge() = default;

    /**
     * @brief HBridge input class to hold the input to the HBridge
     * @note This class only modulates the high side of the H bridge. The low side is assumed to be controlled by a GPIO to reduce
     * complexity.
     */
    class HBridgeInput {
       public:
        /**
         * Duty cycle for the high side of the A phase PWM
         */
        float duty_cycle_a_h;
        /**
         * @brief The state of the low side A phase GPIO
         */
        bool low_side_a_gpio_state;
        /**
         * @brief Duty cycle for the high side of the B phase PWM
         */
        float duty_cycle_b_h;
        /**
         * @brief The state of the low side B phase GPIO
         */
        bool low_side_b_gpio_state;

        /**
         * @brief Equality operator
         * @param other The other HBridgeInput to compare to
         * @return True if the HBridgeInputs are equal, false otherwise
         * @note This is used to determine if the HBridgeInput has changed and needs to be updated or for unit testing
         */
        bool operator==(const HBridgeInput& other) const {
            const bool low_side_gpios_equal =
                (low_side_a_gpio_state == other.low_side_a_gpio_state) && (low_side_b_gpio_state == other.low_side_b_gpio_state);
            // Use the very small number for floating comparisons
            const bool duty_cycles_equal = (fabs(duty_cycle_a_h - other.duty_cycle_a_h) < math::ACCEPTABLE_FLOAT_ERROR) &&
                                           (fabs(duty_cycle_b_h - other.duty_cycle_b_h) < math::ACCEPTABLE_FLOAT_ERROR);

            return low_side_gpios_equal && duty_cycles_equal;
        }
    };

    /**
     * @brief Run the HBridge at a given speed
     * @param input The input to the HBridge
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E run(HBridgeInput input) = 0;

    /**
     * @brief Get the current through the HBridge
     * @param current The current through the HBridge
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E get_current(float& current) = 0;

    /**
     * @brief Get the voltage across the HBridge
     * @param voltage The voltage across the HBridge
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E get_bus_voltage(float& voltage) = 0;
};

}  // namespace hwbridge

#endif