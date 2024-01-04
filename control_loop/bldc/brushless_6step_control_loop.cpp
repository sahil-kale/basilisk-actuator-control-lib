#include "brushless_6step_control_loop.hpp"

#include "util.hpp"

namespace control_loop {

void Brushless6StepControlLoop::init(void) {
    // Reset the status
    status_.reset();
    // Reset the rotor position estimator
    rotor_position_estimator_.reset_estimation();
}

ControlLoop::ControlLoopBaseStatus Brushless6StepControlLoop::run(float speed) {
    do {
        // Poll the phase voltages
        hwbridge::Bridge3Phase::phase_voltage_t phase_voltage;
        auto phase_voltage_read_status = motor_.read_phase_voltage(phase_voltage);

        const bool phase_voltage_read_failed = (phase_voltage_read_status != app_hal_status_E::APP_HAL_OK);
        // Set the warning if the phase voltage read failed
        status_.set_warning(Brushless6StepControlLoop::Brushless6StepWarning::PHASE_VOLTAGE_READ_FAILURE,
                            phase_voltage_read_failed);

        // Attempt to update the rotor position estimator
        bldc_rotor_estimator::ElectricalRotorPosEstimator::EstimatorInputs inputs;
        inputs.phase_voltage = phase_voltage;

        auto rotor_position_estimator_status = rotor_position_estimator_.update(inputs);
        bool rotor_position_estimator_failed = (rotor_position_estimator_status != app_hal_status_E::APP_HAL_OK);
        status_.set_error(Brushless6StepControlLoop::Brushless6StepError::ROTOR_POSITION_ESTIMATOR_UPDATE_FAILURE,
                          rotor_position_estimator_failed);
        if (rotor_position_estimator_failed) {
            break;
        }

        // Get the rotor position
        float rotor_position;
        auto rotor_position_get_status = rotor_position_estimator_.get_rotor_position(rotor_position);
        rotor_position_estimator_failed = rotor_position_get_status != app_hal_status_E::APP_HAL_OK;

        // Set the error if the rotor position estimator failed
        status_.set_error(Brushless6StepControlLoop::Brushless6StepError::ROTOR_POSITION_ESTIMATOR_GET_ANGLE_FAILURE,
                          rotor_position_estimator_failed);
        if (rotor_position_estimator_failed) {
            break;
        }

        // Get the commutation step for the current rotor position
        auto commutation_step = Bldc6Step::determine_commutation_step_from_theta(rotor_position);

        // Determine the duty cycles for the current commutation step
        Bldc6Step::determine_inverter_duty_cycles_trap(phase_command_, commutation_step, speed);

    } while (false);

    // If the status is errored, zero the duty cycles
    if (status_.status == ControlLoop::ControlLoopBaseStatus::ERROR) {
        hwbridge::Bridge3Phase::phase_command_t zero_duty_cycle;
        zero_duty_cycle.duty_cycle_high_side = 0.0f;
        zero_duty_cycle.invert_low_side = false;

        phase_command_[0] = zero_duty_cycle;
        phase_command_[1] = zero_duty_cycle;
        phase_command_[2] = zero_duty_cycle;
    }

    auto bridge_status = motor_.set_phase(phase_command_[0], phase_command_[1], phase_command_[2]);
    bool bridge_failed = (bridge_status != app_hal_status_E::APP_HAL_OK);
    status_.set_error(Brushless6StepControlLoop::Brushless6StepError::BRIDGE_DUTY_CYCLE_SET_FAILURE, bridge_failed);

    return status_.status;
}
}  // namespace control_loop