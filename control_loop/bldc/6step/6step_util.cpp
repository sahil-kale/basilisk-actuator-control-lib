#include "6step_util.hpp"

#include <cmath>

#include "math_util.hpp"

namespace control_loop {
namespace Bldc6Step {

commutation_step_t determine_commutation_step_from_theta(float electrical_theta) {
    commutation_step_t commutation_step = {CommutationSignal::LOW, CommutationSignal::LOW, CommutationSignal::LOW};

    static constexpr float sector_size_radians = 2.0f * math::M_PI_FLOAT / static_cast<float>(num_commutation_steps);
    static constexpr float sector_size_radians_half = sector_size_radians / 2.0f;

    // Determine the commutation step from the electrical theta via if-else ladder
    if ((electrical_theta <= sector_size_radians_half) || (electrical_theta >= (sector_size_radians * 5.0))) {
        // Commutation step 0
        commutation_step = commutation_steps[0];
    } else if ((electrical_theta > sector_size_radians_half) && (electrical_theta <= (sector_size_radians * 1.5))) {
        // Commutation step 1
        commutation_step = commutation_steps[1];
    } else if ((electrical_theta > (sector_size_radians * 1.5)) && (electrical_theta <= (sector_size_radians * 2.5))) {
        // Commutation step 2
        commutation_step = commutation_steps[2];
    } else if ((electrical_theta > (sector_size_radians * 2.5)) && (electrical_theta <= (sector_size_radians * 3.5))) {
        // Commutation step 3
        commutation_step = commutation_steps[3];
    } else if ((electrical_theta > (sector_size_radians * 3.5)) && (electrical_theta <= (sector_size_radians * 4.5))) {
        // Commutation step 4
        commutation_step = commutation_steps[4];
    } else if ((electrical_theta > (sector_size_radians * 4.5)) && (electrical_theta <= (sector_size_radians * 5.5))) {
        // Commutation step 5
        commutation_step = commutation_steps[5];
    } else {
        // do nothing
    }

    return commutation_step;
}

void determine_inverter_duty_cycles_trap(hwbridge::Bridge3Phase::phase_command_t phase_command[3],
                                         Bldc6Step::commutation_step_t current_commutation_step, float motor_speed) {
    // Clamp the motor speed to -1.0f to 1.0f after first abs'ing it
    float abs_speed = fabsf(motor_speed);
    math::clamp(abs_speed, 0.0f, 1.0f);
    for (int i = 0; i < 3; i++) {
        if (current_commutation_step.signals[i] == Bldc6Step::CommutationSignal::HIGH) {
            // Because we contract the phase command as a complementary switching pair, '0.5' is actually 0V phase-to-neutral
            // And the motor speed mapping can be thought of as commanding 'max' phase-to-neutral voltage
            // As a result, we map the motor speed to the duty cycle as follows:
            // | motor_speed | duty_cycle |
            // | 0.0f        | 0.5f       |
            // | 1.0f        | 1.0f       |
            const float duty_cycle = (abs_speed + 1.0f) / 2.0f;

            phase_command[i].duty_cycle_high_side = duty_cycle;
            phase_command[i].enable = true;
        } else if (current_commutation_step.signals[i] == Bldc6Step::CommutationSignal::LOW) {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].enable = true;
        } else {
            phase_command[i].duty_cycle_high_side = 0.0f;
            phase_command[i].enable = false;
        }
    }
}

}  // namespace Bldc6Step
}  // namespace control_loop