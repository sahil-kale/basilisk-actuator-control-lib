#ifndef MOCK_HAL_ADC_HPP
#define MOCK_HAL_ADC_HPP
#include "gmock/gmock.h"
#include "hal_adc.hpp"

namespace basilisk_hal {
class MOCK_HAL_ADC : public HAL_ADC {
   public:
    MOCK_METHOD(app_hal_status_E, read_adc, (float& result, uint8_t channel), (override));
};
}  // namespace basilisk_hal

#endif