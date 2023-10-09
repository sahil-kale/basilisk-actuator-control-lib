#ifndef MOCK_ROTOR_ESTIMATOR_HPP
#define MOCK_ROTOR_ESTIMATOR_HPP

#include "gmock/gmock.h"
#include "rotor_estimator.hpp"

namespace bldc_rotor_estimator {

// Define a mock class for a 3-phase rotor sector sensor
class MOCK_ROTOR_SECTOR_SENSOR : public BldcRotorSectorSensor {
   public:
    MOCK_METHOD(app_hal_status_E, init, (), (override));
    MOCK_METHOD(app_hal_status_E, get_sector, (uint8_t & sector), (override));
};
}  // namespace bldc_rotor_estimator

#endif  // MOCK_ROTOR_ESTIMATOR_HPP