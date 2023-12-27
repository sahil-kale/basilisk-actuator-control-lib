#ifndef BRUSHLESS_6STEP_COMMUTATION_HPP
#define BRUSHLESS_6STEP_COMMUTATION_HPP

#include "bridge_3phase.hpp"

namespace control_loop {

namespace Bldc6Step {
enum class CommutationSignal {
    HIGH,
    LOW,
    Z_RISING,
    Z_FALLING,
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

};  // namespace Bldc6Step

}  // namespace control_loop

#endif  // BRUSHLESS_6STEP_COMMUTATION_HPP