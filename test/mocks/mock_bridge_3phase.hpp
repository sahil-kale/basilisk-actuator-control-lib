#ifndef MOCK_BRIDGE_3PHASE_HPP
#define MOCK_BRIDGE_3PHASE_HPP

#include "bridge_3phase.hpp"
#include "bridge_3phase_drv8323.hpp"
#include "gmock/gmock.h"

namespace hwbridge {
// Define a mock class for the 3-phase bridge
class MOCK_BRIDGE_3PHASE : public Bridge3Phase {
   public:
    MOCK_METHOD(void, set_phase, (const phase_command_t& u, const phase_command_t& v, const phase_command_t& w), (override));
    MOCK_METHOD(app_hal_status_E, init, (), (override));
    MOCK_METHOD(app_hal_status_E, read_bemf, (bemf_voltage_t & bemf_voltage), (override));
    MOCK_METHOD(app_hal_status_E, read_current, (phase_current_t & current), (override));
    MOCK_METHOD(app_hal_status_E, read_bus_voltage, (float& voltage), (override));
};

}  // namespace hwbridge
#endif  // MOCK_BRIDGE_3PHASE_HPP