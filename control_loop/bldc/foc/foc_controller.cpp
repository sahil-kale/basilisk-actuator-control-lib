#include "foc_controller.hpp"

namespace control_loop {
namespace BldcFoc {

void FOCController::init(float kp, float ki) {
    pid_direct_current_.set_kp(kp);
    pid_direct_current_.set_ki(ki);
    pid_direct_current_.reset();

    pid_quadrature_current_.set_kp(kp);
    pid_quadrature_current_.set_ki(ki);
    pid_quadrature_current_.reset();

    // Reset the FOC frame vars
    foc_frame_vars_ = FOCFrameVars();
}

FOCController::ControlLoopType FOCController::get_desired_control_loop_type(bool theta_valid) {
    ControlLoopType desired_control_loop_type = ControlLoopType::OPEN_LOOP;
    if (theta_valid) {
        desired_control_loop_type = ControlLoopType::CLOSED_LOOP;
    }
    return desired_control_loop_type;
}

FOCController::FOCFrameVars FOCController::run_foc(FOCController::FOCInputs foc_inputs,
                                                   hwbridge::Bridge3Phase::phase_command_t phase_commands[3]) {
    do {
        foc_frame_vars_.control_loop_type = get_desired_control_loop_type(foc_inputs.rotor_position_valid);
        switch (foc_frame_vars_.control_loop_type) {
            case FOCController::ControlLoopType::OPEN_LOOP: {
                // Advance the angle
                foc_frame_vars_.commanded_rotor_theta = advance_open_loop_angle(
                    foc_frame_vars_.commanded_rotor_theta, foc_inputs.open_loop_theta_velocity, foc_inputs.dt);
                // Make the Vq equal to the param'ed value for drive voltage
                foc_frame_vars_.V_direct_quad.quadrature = foc_inputs.open_loop_quadrature_voltage;
                // Make the Vd equal to 0
                foc_frame_vars_.V_direct_quad.direct = 0.0f;
            } break;
            case FOCController::ControlLoopType::CLOSED_LOOP: {
                // Run the PI controller
                const float q_voltage_delta = pid_quadrature_current_.calculate(foc_inputs.i_direct_quad.quadrature,
                                                                                foc_inputs.i_direct_quad_ref.quadrature);
                const float d_voltage_delta =
                    pid_direct_current_.calculate(foc_inputs.i_direct_quad.direct, foc_inputs.i_direct_quad_ref.direct);
                foc_frame_vars_.V_direct_quad.quadrature += q_voltage_delta;
                foc_frame_vars_.V_direct_quad.direct += d_voltage_delta;

                // Clamp the Vq and Vd
                foc_frame_vars_.V_direct_quad = clamp_Vdq(foc_frame_vars_.V_direct_quad, foc_inputs.bus_voltage);

                // Keep this around for the open loop case
                foc_frame_vars_.commanded_rotor_theta = foc_inputs.theta_e;
            } break;
            default:
                break;
        }

        // Determine the appropriate duty cycles for the inverter
        FocDutyCycleResult result =
            determine_inverter_duty_cycles_foc(foc_frame_vars_.commanded_rotor_theta, foc_frame_vars_.V_direct_quad,
                                               foc_inputs.bus_voltage, foc_inputs.pwm_control_type, phase_commands);

        // Set the debug vars
        foc_frame_vars_.foc_inputs = foc_inputs;
        foc_frame_vars_.duty_cycle_result = result;

    } while (false);

    return foc_frame_vars_;
}

}  // namespace BldcFoc
}  // namespace control_loop