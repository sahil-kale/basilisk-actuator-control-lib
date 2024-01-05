#include "foc_util.hpp"
#include "gtest/gtest.h"
#include "math.h"
#include "math_util.hpp"

using namespace ::testing;

namespace control_loop {
namespace BldcFoc {

TEST(BldcFocUtils, test_theta_valpha_beta) {
    // Get the alpha and beta components of the voltage vector
    float Vdirect = 1.0f;
    float Vquadrature = 0.0f;
    math::direct_quad_t V_direct_quad = {Vdirect, Vquadrature};
    float theta = 0.0f;
    float bus_voltage = 1.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;

    // Create an array of phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_commands[3];

    // Determine the duty cycles
    FocDutyCycleResult result =
        determine_inverter_duty_cycles_foc(theta, V_direct_quad, bus_voltage, pwm_control_type, phase_commands);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_commands[0].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_commands[1].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_commands[2].duty_cycle_high_side);
}

// Test the bus voltage is 0 with sine pwm
TEST(BldcFocUtils, test_theta_valpha_beta_bus_voltage_0_sine) {
    // Get the alpha and beta components of the voltage vector
    float Vdirect = 1.0f;
    float Vquadrature = 0.0f;
    math::direct_quad_t V_direct_quad = {Vdirect, Vquadrature};
    float theta = 0.0f;
    float bus_voltage = 0.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SINE;

    // Create an array of phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_commands[3];

    // Determine the duty cycles
    FocDutyCycleResult result =
        determine_inverter_duty_cycles_foc(theta, V_direct_quad, bus_voltage, pwm_control_type, phase_commands);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_commands[0].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_commands[1].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_commands[2].duty_cycle_high_side);

    // Test that the duty cycles are 0
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, 0.0f);
}

// Test the bus voltage is 0 with svpwm
TEST(BldcFocUtils, test_theta_valpha_beta_bus_voltage_0_svpwm) {
    // Get the alpha and beta components of the voltage vector
    float Vdirect = 1.0f;
    float Vquadrature = 0.0f;
    math::direct_quad_t V_direct_quad = {Vdirect, Vquadrature};
    float theta = 0.0f;
    float bus_voltage = 0.0f;
    BrushlessFocPwmControlType pwm_control_type = BrushlessFocPwmControlType::SPACE_VECTOR;

    // Create an array of phase commands
    hwbridge::Bridge3Phase::phase_command_t phase_commands[3];

    // Determine the duty cycles
    FocDutyCycleResult result =
        determine_inverter_duty_cycles_foc(theta, V_direct_quad, bus_voltage, pwm_control_type, phase_commands);

    // Check the alpha and beta components of the voltage vector
    EXPECT_FLOAT_EQ(result.V_alpha_beta.alpha, 1.0f);
    EXPECT_FLOAT_EQ(result.V_alpha_beta.beta, 0.0f);

    // Test that the duty cycles are equal
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, phase_commands[0].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, phase_commands[1].duty_cycle_high_side);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, phase_commands[2].duty_cycle_high_side);

    // Test that the duty cycles are 0
    EXPECT_FLOAT_EQ(result.duty_cycle_u_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_v_h, 0.0f);
    EXPECT_FLOAT_EQ(result.duty_cycle_w_h, 0.0f);
}

// Test the SVM sectors
TEST(BldcFocUtils, test_svm_sectors) {
    math::alpha_beta_t V_alpha_beta;
    // Test sector 1 (theta = 0)
    V_alpha_beta.alpha = 1.0f;
    V_alpha_beta.beta = 0.0f;
    EXPECT_EQ(svm_sector(V_alpha_beta), 1);

    // Test sector 2 (theta = pi)
    V_alpha_beta.alpha = 0.0f;
    V_alpha_beta.beta = 1.0f;
    EXPECT_EQ(svm_sector(V_alpha_beta), 2);

    // Test sector 3 (theta = 5pi/6)
    V_alpha_beta.alpha = cos(5.0f * M_PI / 6.0f);
    V_alpha_beta.beta = sin(5.0f * M_PI / 6.0f);
    EXPECT_EQ(svm_sector(V_alpha_beta), 3);

    // Test sector 4 (theta = 7pi/6)
    V_alpha_beta.alpha = cos(7.0f * M_PI / 6.0f);
    V_alpha_beta.beta = sin(7.0f * M_PI / 6.0f);
    EXPECT_EQ(svm_sector(V_alpha_beta), 4);

    // Test sector 5 (theta = 3pi/2)
    V_alpha_beta.alpha = 0.0f;
    V_alpha_beta.beta = -1.0f;
    EXPECT_EQ(svm_sector(V_alpha_beta), 5);

    // Test sector 6 (theta = 11pi/6)
    V_alpha_beta.alpha = cos(11.0f * M_PI / 6.0f);
    V_alpha_beta.beta = sin(11.0f * M_PI / 6.0f);
    EXPECT_EQ(svm_sector(V_alpha_beta), 6);
}

// Test the sine pwm generation
TEST(BldcFocUtils, test_sine_pwm) {
    math::abc_t duty_cycles;
    math::alpha_beta_t V_alpha_beta;
    const float Vbus = 2.0f;  // This allows us to make the magnitude of V_alpha_beta = 1.0f

    // In general, sine PWM should output the following
    // a = (0.5 * Vbus * cos(theta)) + 1.0f)/2.0f
    // b = (0.5 * Vbus * cos(theta + 2pi/3)) + 1.0f)/2.0f
    // c = (0.5 * Vbus * cos(theta + 4pi/3)) + 1.0f)/2.0f

    // For the case of Vbus = 2.0f, this simplifies to
    // a = (cos(theta) + 1.0f)/2.0f
    // b = (cos(theta + 2pi/3) + 1.0f)/2.0f
    // c = (cos(theta + 4pi/3) + 1.0f)/2.0f

    // NOTE: theta is not the same as the electrical angle theta_e. It is the angle of the voltage vector in the alpha/beta frame
    // hence the use of 'cos' instead of 'sin' in the equations above

    // Test theta = 0
    V_alpha_beta.alpha = cos(0.0f);
    V_alpha_beta.beta = sin(0.0f);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 1.0f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 0.25f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 0.25f);

    // Test theta = pi/3
    V_alpha_beta.alpha = cos(M_PI / 3.0f);
    V_alpha_beta.beta = sin(M_PI / 3.0f);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 0.75f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 0.75f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 0.0f);

    // Test theta = 2pi/3
    V_alpha_beta.alpha = cos(2.0f * M_PI / 3.0f);
    V_alpha_beta.beta = sin(2.0f * M_PI / 3.0f);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 0.25f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 1.0f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 0.25f);

    // Test theta = pi
    V_alpha_beta.alpha = cos(M_PI);
    V_alpha_beta.beta = sin(M_PI);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 0.0f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 0.75f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 0.75f);

    // Test theta = 4pi/3
    V_alpha_beta.alpha = cos(4.0f * M_PI / 3.0f);
    V_alpha_beta.beta = sin(4.0f * M_PI / 3.0f);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 0.25f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 0.25f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 1.0f);

    // Test theta = 5pi/3
    V_alpha_beta.alpha = cos(5.0f * M_PI / 3.0f);
    V_alpha_beta.beta = sin(5.0f * M_PI / 3.0f);
    duty_cycles = sine_pwm(V_alpha_beta, Vbus);

    EXPECT_FLOAT_EQ(duty_cycles.a, 0.75f);
    EXPECT_FLOAT_EQ(duty_cycles.b, 0.0f);
    EXPECT_FLOAT_EQ(duty_cycles.c, 0.75f);
}

// Test the open loop angle advancing
TEST(BldcFocUtils, test_open_loop_angle_advancing) {
    float theta = 0.0f;
    float omega = 1.0f;
    float dt = 1.0f;

    EXPECT_FLOAT_EQ(advance_open_loop_angle(theta, omega, dt), 1.0f);

    // Test wrapping around from 3PI/2 to PI/3 (delta of 5PI/6)
    theta = 3.0f * M_PI / 2.0f;
    omega = 5 * M_PI / 6;
    dt = 1.0f;
    EXPECT_FLOAT_EQ(advance_open_loop_angle(theta, omega, dt), M_PI / 3.0f);

    // Test wrapping down (pi/3 to 3pi/2)
    theta = M_PI / 3.0f;
    omega = -5 * M_PI / 6;
    dt = 1.0f;
    EXPECT_FLOAT_EQ(advance_open_loop_angle(theta, omega, dt), 3.0f * M_PI / 2.0f);
}

}  // namespace BldcFoc
}  // namespace control_loop