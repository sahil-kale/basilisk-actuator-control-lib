#include "brushless_foc.hpp"

#include "gtest/gtest.h"
#include "math_util.hpp"

using namespace ::testing;

namespace control_loop {
namespace BldcFoc {

TEST(BldcFoc, test_theta_valpha_beta) {
    // Get the alpha and beta components of the voltage vector
    math::dq_pair_t V_dq;
    V_dq.direct = 1.0f;
    V_dq.quadrature = 0.0f;
    float theta = 0.0f;
    float bus_voltage = 1.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;

    // Create the phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_command_u;
    hwbridge::Bridge3Phase::phase_command_t phase_command_v;
    hwbridge::Bridge3Phase::phase_command_t phase_command_w;

    // Determine the duty cycles
    FocDutyCycleResult result = determine_inverter_duty_cycles_foc(theta, V_dq, bus_voltage, pwm_control_type, phase_command_u,
                                                                   phase_command_v, phase_command_w);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_command_u.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_command_v.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_command_w.duty_cycle_high_side);
}

// Test the bus voltage is 0 with sine pwm
TEST(BldcFoc, test_theta_valpha_beta_bus_voltage_0_sine) {
    // Get the alpha and beta components of the voltage vector
    math::dq_pair_t V_dq;
    V_dq.direct = 1.0f;
    V_dq.quadrature = 0.0f;
    float theta = 0.0f;
    float bus_voltage = 0.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;

    // Create the phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_command_u;
    hwbridge::Bridge3Phase::phase_command_t phase_command_v;
    hwbridge::Bridge3Phase::phase_command_t phase_command_w;

    // Determine the duty cycles
    FocDutyCycleResult result = determine_inverter_duty_cycles_foc(theta, V_dq, bus_voltage, pwm_control_type, phase_command_u,
                                                                   phase_command_v, phase_command_w);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_command_u.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_command_v.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_command_w.duty_cycle_high_side);

    // Test that the duty cycles are 0
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, 0.0f);
}

// Test the bus voltage is 0 with svpwm
TEST(BldcFoc, test_theta_valpha_beta_bus_voltage_0_svpwm) {
    // Get the alpha and beta components of the voltage vector
    math::dq_pair_t V_dq;
    V_dq.direct = 1.0f;
    V_dq.quadrature = 0.0f;
    float theta = 0.0f;
    float bus_voltage = 0.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SPACE_VECTOR;

    // Create the phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_command_u;
    hwbridge::Bridge3Phase::phase_command_t phase_command_v;
    hwbridge::Bridge3Phase::phase_command_t phase_command_w;

    // Determine the duty cycles
    FocDutyCycleResult result = determine_inverter_duty_cycles_foc(theta, V_dq, bus_voltage, pwm_control_type, phase_command_u,
                                                                   phase_command_v, phase_command_w);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_command_u.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_command_v.duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_command_w.duty_cycle_high_side);

    // Test that the duty cycles are 0
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, 0.0f);
}
}  // namespace BldcFoc
}  // namespace control_loop