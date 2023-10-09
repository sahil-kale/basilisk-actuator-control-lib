#ifndef BRIDGE_3PHASE_HPP
#define BRIDGE_3PHASE_HPP

#include "hal_common.hpp"

namespace hwbridge {

// Define a generic 3-phase bridge class

class Bridge3Phase {
   public:
    Bridge3Phase() = default;
    virtual ~Bridge3Phase() = default;

    typedef struct phase_command {
        float duty_cycle_high_side;
        bool invert_low_side;
    } phase_command_t;

    // Define a struct to return the PHASE (not backemf) voltage
    typedef struct bemf_voltage {
        float u;
        float v;
        float w;
    } bemf_voltage_t;

    // Define a struct to return the current
    typedef struct current {
        float u;
        float v;
        float w;
    } phase_current_t;

    virtual app_hal_status_E init() = 0;

    // Define a virtual function to set the individual phases' duty cycles and enable/disable the phase
    virtual void set_phase(const phase_command_t& u, const phase_command_t& v, const phase_command_t& w) = 0;

    // Define a virtual function to get the back emf voltage
    virtual app_hal_status_E read_bemf(bemf_voltage_t& bemf_voltage) = 0;

    // Define a virtual function to get the current
    virtual app_hal_status_E read_current(phase_current_t& current) = 0;

    // Define a virtual function to get the bus voltage
    virtual app_hal_status_E read_bus_voltage(float& bus_voltage) = 0;

    static constexpr uint8_t NUM_PHASES = 3;
};

}  // namespace hwbridge

#endif  // BRIDGE_3PHASE_HPP