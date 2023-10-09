#ifndef BRUSHED_CONTROL_LOOP_HPP
#define BRUSHED_CONTROL_LOOP_HPP
#include <stdint.h>

#include "control_loop.hpp"
#include "hal_clock.hpp"
#ifdef UNIT_TEST
// forward declare the BrushedControlLoopTest class
class BrushedControlLoopTest;
#endif

namespace control_loop {

// Define a class BrushedControlLoop that inherits from ControlLoop

class BrushedControlLoop : public ControlLoop {
   public:
    BrushedControlLoop();

    // Define a run method that overrides the run method in ControlLoop
    void run(float speed) override;

    // Function to register a timer with the control loop

    // Function to register a HAL clock with the control loop
    void register_clock(basilisk_hal::HAL_CLOCK* clock) { clock_ = clock; }

    // 0.0 -> 1.0f
    typedef struct h_bridge_motor_speed_outputs_t {
        float DC_A_HIGH;
        float DC_A_LOW;
        float DC_B_HIGH;
        float DC_B_LOW;
    } h_bridge_motor_speed_outputs_t;

   private:
    /**
     * @brief Computes the PWM duty cycle of the 4 pins based on desired motor speed (-1.0f->1.0f)
     * and whether brake mode is enabled or not
     *
     * @param motor_speed
     * @param brake_mode
     */
    h_bridge_motor_speed_outputs_t compute_motor_speed_outputs(float motor_speed, bool brake_mode, utime_t current_time_us);

    utime_t last_speed_dir_change_time_us_{0};
    float last_motor_speed_{0.0f};
    basilisk_hal::HAL_CLOCK* clock_{nullptr};

#ifdef UNIT_TEST
    friend class BrushedControlLoopTest;
#endif
};
}  // namespace control_loop

#endif