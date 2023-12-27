#include "brushless_6step_commutation.hpp"

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
}  // namespace Bldc6Step
}  // namespace control_loop