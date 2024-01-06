#include "foc_controller.hpp"

#include "gtest/gtest.h"
#include "mock_hal_clock.hpp"

using namespace ::testing;

class FOCControllerPublic : public control_loop::BldcFoc::FOCController {
   public:
    basilisk_hal::MOCK_HAL_CLOCK mock_clock;
    FOCControllerPublic() : FOCController(mock_clock) {}
    FOCControllerPublic(basilisk_hal::HAL_CLOCK& clock) : FOCController(clock) {}
    using FOCController::get_desired_control_loop_type;
};

namespace control_loop {
namespace BldcFoc {
TEST(FOCControllerTest, GetDesiredControlLoopType) {
    // Create an FOC controller public object
    FOCControllerPublic foc_controller_public;
    EXPECT_EQ(foc_controller_public.get_desired_control_loop_type(false), FOCController::ControlLoopType::OPEN_LOOP);
    EXPECT_EQ(foc_controller_public.get_desired_control_loop_type(true), FOCController::ControlLoopType::CLOSED_LOOP);
}

}  // namespace BldcFoc
}  // namespace control_loop