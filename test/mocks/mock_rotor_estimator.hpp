#ifndef MOCK_ROTOR_ESTIMATOR_HPP
#define MOCK_ROTOR_ESTIMATOR_HPP

#include "gmock/gmock.h"
#include "rotor_estimator.hpp"

namespace bldc_rotor_estimator {

// Define a mock class for a 3-phase rotor sector sensor
class MOCK_ROTOR_SECTOR_SENSOR : public BldcRotorSectorSensor {
   public:
    MOCK_METHOD(app_hal_status_E, get_electrical_angle, (float& angle), (override));
    MOCK_METHOD(app_hal_status_E, reset, (), (override));
};

// Define a mock class for an absolute rotor sensor

class MOCK_ROTOR_ABSOLUTE_SENSOR : public ElectricalRotorPosEstimator {
   public:
    MOCK_METHOD(app_hal_status_E, update, (const EstimatorInputs& inputs), (override));
    MOCK_METHOD(app_hal_status_E, get_rotor_position, (float& rotor_position), (override));
    MOCK_METHOD(app_hal_status_E, get_rotor_velocity, (float& rotor_velocity), (override));
    MOCK_METHOD(app_hal_status_E, reset_estimation, (), (override));
    MOCK_METHOD(bool, is_estimation_valid, (), (override));
};

}  // namespace bldc_rotor_estimator

#endif  // MOCK_ROTOR_ESTIMATOR_HPP