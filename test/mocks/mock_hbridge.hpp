#ifndef MOCK_H_BRIDGE_HPP
#define MOCK_H_BRIDGE_HPP

#include "bridge_hbridge.hpp"
#include "gmock/gmock.h"

namespace hwbridge {
// Define a mock class for an HBridge
class MOCK_HBRIDGE : public HBridge {
   public:
    MOCK_METHOD(app_hal_status_E, run, (HBridgeInput input), (override));
    MOCK_METHOD(app_hal_status_E, get_current, (float& current), (override));
    MOCK_METHOD(app_hal_status_E, get_voltage, (float& voltage), (override));
};

}  // namespace hwbridge

#endif  // MOCK_H_BRIDGE_HPP