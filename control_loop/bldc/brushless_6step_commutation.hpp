#ifndef BRUSHLESS_6STEP_COMMUTATION_HPP
#define BRUSHLESS_6STEP_COMMUTATION_HPP

#include "bridge_3phase.hpp"

namespace control_loop {

namespace Bldc6Step {
enum class CommutationSignal {
    HIGH,       // The phase's DC voltage is set to high (desired voltage)
    LOW,        // The phase's DC voltage is set to low (ground)
    Z_RISING,   // The phase is set to high impedance (floating) and the voltage is rising (used for zero crossing detection)
    Z_FALLING,  // The phase is set to high impedance (floating) and the voltage is falling (used for zero crossing detection)
};

typedef union {
    CommutationSignal signals[hwbridge::Bridge3Phase::NUM_PHASES];
    struct {
        CommutationSignal u;
        CommutationSignal v;
        CommutationSignal w;
    } phase_commutation_signals;
} commutation_step_t;

constexpr uint8_t num_commutation_steps = 6;
constexpr commutation_step_t commutation_steps[num_commutation_steps] = {
    {CommutationSignal::Z_FALLING, CommutationSignal::HIGH, CommutationSignal::LOW},
    {CommutationSignal::LOW, CommutationSignal::HIGH, CommutationSignal::Z_RISING},
    {CommutationSignal::LOW, CommutationSignal::Z_FALLING, CommutationSignal::HIGH},
    {CommutationSignal::Z_RISING, CommutationSignal::LOW, CommutationSignal::HIGH},
    {CommutationSignal::HIGH, CommutationSignal::LOW, CommutationSignal::Z_FALLING},
    {CommutationSignal::HIGH, CommutationSignal::Z_RISING, CommutationSignal::LOW},
};

/**
 * @brief Determine the commutation step from the electrical theta
 * @param electrical_theta The electrical theta (radians)
 * @return The commutation step
 */
commutation_step_t determine_commutation_step_from_theta(float electrical_theta);

/**
 * @brief Generate the duty cycles for the 3 phases based on the commutation step
 * @param phase_command array of phase commands to be filled in, u-v-w
 * @param commutation_step the commutation step to generate the duty cycles for
 * @param motor_speed the motor speed to generate the duty cycles for. 0 - 1.0f
 */
void determine_inverter_duty_cycles_trap(hwbridge::Bridge3Phase::phase_command_t phase_command[3],
                                         Bldc6Step::commutation_step_t current_commutation_step, float motor_speed);

};  // namespace Bldc6Step

}  // namespace control_loop

#endif  // BRUSHLESS_6STEP_COMMUTATION_HPP