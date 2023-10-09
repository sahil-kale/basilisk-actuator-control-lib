#ifndef HAL_ADC_HPP
#define HAL_ADC_HPP

#include <stdint.h>

#include "hal_common.hpp"

namespace basilisk_hal {
class HAL_ADC {
   public:
    HAL_ADC() = default;
    virtual ~HAL_ADC() {}
    /**
     * @brief Read the ADC value of a given channel
     * @param channel The channel to read from
     * @return The voltage value read at the given channel
     */
    virtual app_hal_status_E read_adc(float& result, uint8_t channel) = 0;
};
}  // namespace basilisk_hal

#endif