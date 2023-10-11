#ifndef BRIDGE_HBRIDGE_HPP
#define BRIDGE_HBRIDGE_HPP
#include "hal_common.hpp"
#include "math.h"
#include "math_util.hpp"

namespace hwbridge {

// Define a generic h-bridge class
class HBridge {
   public:
    HBridge() = default;
    virtual ~HBridge() = default;

    class HBridgeInput {
       public:
        float duty_cycle_a_h;
        bool low_side_a_gpio_state;
        float duty_cycle_b_h;
        bool low_side_b_gpio_state;

        // Overload the == operator
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
     * @param speed The speed to run the HBridge at
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
    virtual app_hal_status_E get_voltage(float& voltage) = 0;
};

}  // namespace hwbridge

#endif