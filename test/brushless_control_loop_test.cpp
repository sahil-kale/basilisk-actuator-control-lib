#include "brushless_control_loop.hpp"

#include "bridge_3phase.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "math.h"
#include "math_foc.hpp"
#include "mock_bridge_3phase.hpp"
#include "mock_hal_clock.hpp"
#include "mock_rotor_estimator.hpp"

namespace control_loop {
using namespace ::testing;

NiceMock<basilisk_hal::MOCK_HAL_CLOCK> mock_clock;

class BrushlessControlLoopTest : public BrushlessControlLoop {
   public:
    hwbridge::MOCK_BRIDGE_3PHASE bridge;

    BrushlessControlLoop::BrushlessFocControLoopParams foc_params_{.current_control_bandwidth_rad_per_sec = 0.0f,

                                                                   .phase_resistance = 0.0f,
                                                                   .phase_inductance = 0.0f,
                                                                   .pm_flux_linkage = 0.0f,

                                                                   .foc_start_timeout_period_us = 0,
                                                                   .disable_ki = false,

                                                                   .speed_to_iq_gain = 0.0f,
                                                                   .i_d_reference_default = 0.0f,

                                                                   .current_lpf_fc = 0.0f,

                                                                   .pwm_control_type = BldcFoc::BrushlessFocPwmControlType::SINE};

    BrushlessControlLoop::BrushlessControlLoopParams test_params_{
        .commutation_type = BrushlessControlLoop::BrushlessControlLoopCommutationType::TRAPEZOIDAL,
        .foc_params = foc_params_,
        .open_loop_full_speed_theta_velocity = 0.0f};

    BrushlessControlLoopTest(bldc_rotor_estimator::ElectricalRotorPosEstimator& rotor_position_estimator,
                             basilisk_hal::HAL_CLOCK& clock)
        : BrushlessControlLoop(bridge, clock, rotor_position_estimator) {}

    // Make the private functions public so we can test them
    using BrushlessControlLoop::get_desired_state;
};

// Test the state machine transition from stop to start, and a time out makes it go back to stop
TEST(BrushlessControlLoopTest, test_stop_to_start_to_run) {
    // Create a mock rotor sensor
    NiceMock<bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR> sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::ElectricalRotorPosEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessControlLoop::BrushlessControlLoopState::STOP),
              BrushlessControlLoop::BrushlessControlLoopState::STOP);

    // Call the desired state function with a time of foc_start_timeout_period -1, time at start of 0, and a motor speed of 0.1,
    // with the rotor estimator valid Ensure that the desired state is run
    EXPECT_EQ(test_control_loop.get_desired_state(0.1, BrushlessControlLoop::BrushlessControlLoopState::STOP),
              BrushlessControlLoop::BrushlessControlLoopState::RUN);
}

// Test the state machine transition from run to stop
TEST(BrushlessControlLoopTest, test_run_to_stop) {
    // Create a mock rotor sensor
    bldc_rotor_estimator::MOCK_ROTOR_SECTOR_SENSOR sector_sensor;
    // Initialize a sector sensor from hall
    bldc_rotor_estimator::ElectricalRotorPosEstimatorFromHall rotor_estimator(mock_clock, sector_sensor);

    // instantiate a brushless foc control loop test class
    BrushlessControlLoopTest test_control_loop(rotor_estimator, mock_clock);

    // Call the desired state function with a time of 0, time at start of 0, and a motor speed of 0
    // Ensure that the desired state is stop
    EXPECT_EQ(test_control_loop.get_desired_state(0, BrushlessControlLoop::BrushlessControlLoopState::RUN),
              BrushlessControlLoop::BrushlessControlLoopState::STOP);
}

}  // namespace control_loop