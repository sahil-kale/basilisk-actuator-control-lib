#include "rotor_estimator.hpp"

#include "brushless_6step_commutation.hpp"
#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"
#include "util.hpp"

namespace bldc_rotor_estimator {

app_hal_status_E BldcSensorlessRotorSectorSensor::reset() {
    estimated_electrical_angle_ = 0.0f;
    time_of_last_commutation_ = clock_.get_time_us();
    return APP_HAL_OK;
}

app_hal_status_E BldcSensorlessRotorSectorSensor::get_electrical_angle(float& angle) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        // Get the bemf voltage
        hwbridge::Bridge3Phase::phase_voltage_t phase_voltage;
        ret = bridge_.read_phase_voltage(phase_voltage);
        const utime_t current_time = clock_.get_time_us();

        if (ret != APP_HAL_OK) {
            break;
        }

        // Get the commutation step as a function of the electrical angle
        control_loop::Bldc6Step::commutation_step_t step =
            control_loop::Bldc6Step::determine_commutation_step_from_theta(estimated_electrical_angle_);

        // Check if a zero crossing has occurred
        const bool zero_crossing = zero_crossing_detected(phase_voltage, step);

        // If we detect a zero crossing, then we should store the time of the zero crossing
        // NOTE: time_of_last_zero_crossing_ is only set to zero when the rotor position estimator is reset
        if (zero_crossing && (time_of_last_zero_crossing_ == 0)) {
            // Store the time of the zero crossing
            time_of_last_zero_crossing_ = current_time;
        }

        if (time_of_last_zero_crossing_ != 0) {
            const utime_t commutation_to_zc_delta = time_of_last_zero_crossing_ - time_of_last_commutation_;
            const utime_t commutation_to_current_time_delta = current_time - time_of_last_commutation_;

            if (commutation_to_current_time_delta >= (commutation_to_zc_delta * 2)) {
                // We should now update the electrical angle
                estimated_electrical_angle_ += 2.0f * math::M_PI_FLOAT / 6.0f;
                // Implement a wraparound
                math::wraparound(estimated_electrical_angle_, 0.0f, float(2.0f * M_PI));

                // Update the time of the last commutation
                time_of_last_commutation_ = current_time;

                // Reset the time of the last zero crossing
                time_of_last_zero_crossing_ = 0;
            }
        }

        // Update the angle value
        angle = estimated_electrical_angle_;

    } while (false);

    return ret;
}

bool BldcSensorlessRotorSectorSensor::zero_crossing_detected(
    const hwbridge::Bridge3Phase::phase_voltage_t& phase_voltage,
    control_loop::Bldc6Step::commutation_step_t current_commutation_step) {
    float phase_sum = 0.0f;
    control_loop::Bldc6Step::CommutationSignal zero_crossing_signal = control_loop::Bldc6Step::CommutationSignal::Z_RISING;
    float undriven_phase_voltage = 0.0f;

    const float phase_voltages[hwbridge::Bridge3Phase::NUM_PHASES] = {phase_voltage.u, phase_voltage.v, phase_voltage.w};

    for (uint8_t i = 0; i < hwbridge::Bridge3Phase::NUM_PHASES; i++) {
        if ((current_commutation_step.signals[i] != control_loop::Bldc6Step::CommutationSignal::Z_FALLING) &&
            (current_commutation_step.signals[i] != control_loop::Bldc6Step::CommutationSignal::Z_RISING)) {
            phase_sum += phase_voltages[i];
        } else {
            zero_crossing_signal = current_commutation_step.signals[i];
            undriven_phase_voltage = phase_voltages[i];
        }
    }

    float zero_crossing_threshold = phase_sum / 2.0f;  // NOTE: This requires the bemf voltage to run when the PWM is ON
    bool return_value = false;
    if (zero_crossing_signal == control_loop::Bldc6Step::CommutationSignal::Z_RISING) {
        if (undriven_phase_voltage > zero_crossing_threshold) {
            return_value = true;
        }
    } else {
        if (undriven_phase_voltage < zero_crossing_threshold) {
            return_value = true;
        }
    }

    return return_value;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::reset_estimation() {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position_ = 0.0f;
    velocity_ = 0.0f;
    velocity_previous_ = 0.0f;
    compensated_velocity_ = 0.0f;
    time_at_last_hall_update_ = 0;
    time_update_last_called_ = 0;
    number_of_hall_updates_ = 0;
    acceleration_ = 0.0f;

    rotor_position_ = raw_hall_angle_;

    return ret;
}

bool ElectricalRotorPosEstimatorFromHall::is_estimation_valid() {
    return true;  // Since we know the absolute position, we should always return true
}

bool ElectricalRotorPosEstimatorFromHall::is_interpolation_permitted() {
    bool ret = false;
    if ((params_ != nullptr) && (params_->enable_interpolation)) {
        const bool num_hall_updates_to_start = (number_of_hall_updates_ >= params_->num_hall_updates_to_start);
        // Also make the return conditional if the position estimate no greater than the param for tolerance
        // compute the angle diff
        float angle_diff = rotor_position_ - raw_hall_angle_;

        // Account for over/underflow
        // NOTE: if the rotor delta theta is greater than pi radians, then this detection will not work
        math::wraparound(angle_diff, -math::M_PI_FLOAT, math::M_PI_FLOAT);

        const bool rotor_position_tolerance = (fabs(angle_diff) <= params_->max_estimate_angle_overrun);

        // We check the velocity to see if it is above the minimum estimation velocity, as well as the compensated velocity in
        // case we think the rotor is decelerating below the minimum estimation velocity
        const bool rotor_velocity_above_min = (fabs(velocity_) >= params_->minimum_estimation_velocity) &&
                                              (fabs(compensated_velocity_) >= params_->minimum_estimation_velocity);

        ret = num_hall_updates_to_start && rotor_position_tolerance && rotor_velocity_above_min;
    }
    return ret;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::init(ElectricalRotorPosEstimatorFromHallParams* params) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        // Set the internal params pointer
        params_ = params;

        // If the params pointer is not null, then we should reset the rotor position estimation
        if (params_ != nullptr) {
            ret = sector_sensor_.get_electrical_angle(raw_hall_angle_);
            if (ret != APP_HAL_OK) {
                break;
            }
            ret = reset_estimation();

        } else {
            ret = APP_HAL_ERROR;
        }

    } while (false);

    return ret;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::update(const ElectricalRotorPosEstimator::EstimatorInputs& inputs) {
    app_hal_status_E ret = APP_HAL_OK;
    const utime_t time = inputs.time;
    do {
        if (params_ == nullptr) {
            ret = APP_HAL_ERROR;
            break;
        }

        float measured_electrical_angle = 0.0f;
        ret = sector_sensor_.get_electrical_angle(measured_electrical_angle);

        if (ret != APP_HAL_OK) {
            break;
        }

        if (is_interpolation_permitted()) {
            const float current_measurement_period =
                (float)(time - time_update_last_called_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;

            // Now, update the compensated velocity with the acceleration estimate
            compensated_velocity_ += acceleration_ * current_measurement_period;

            // Update the rotor position with the velocity estimate
            rotor_position_ += compensated_velocity_ * current_measurement_period;

            // Implement a wraparound
            math::wraparound(rotor_position_, 0.0f, float(2.0f * M_PI));
        } else {
            // Just update the rotor position with the measured electrical angle
            rotor_position_ = measured_electrical_angle;
            // and compensated vel/accl to 0
            compensated_velocity_ = 0.0f;
            acceleration_ = 0.0f;
        }

        float raw_hall_angle = measured_electrical_angle;
        // We should not update velocity estimations if the raw hall angle is the same as the previous one
        if (fabs(raw_hall_angle - raw_hall_angle_) > 0.001f) {
            // Calculate velocity with a simple differentiation
            const float time_delta_since_hall_update =
                (float)(time - time_at_last_hall_update_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;

            float raw_hall_angle_diff = raw_hall_angle - raw_hall_angle_;

            // Account for over/underflow
            // NOTE: if the rotor delta theta is greater than pi radians, then this detection will not work
            math::wraparound(raw_hall_angle_diff, -math::M_PI_FLOAT, math::M_PI_FLOAT);
            velocity_previous_ = velocity_;

            // Calculate the acceleration if the time delta is greater than 0
            if (time_delta_since_hall_update > 0.0f) {
                velocity_ = raw_hall_angle_diff / time_delta_since_hall_update;
                acceleration_ = (velocity_ - velocity_previous_) / time_delta_since_hall_update;
            } else {
                velocity_ = 0.0f;
                acceleration_ = 0.0f;
            }

            compensated_velocity_ = velocity_;

            rotor_position_ = raw_hall_angle;

            // Now, compensate for the rotor position error
            // In a hall sensor, the rotor position is only known to within 60 degrees
            // And the hall sensor reports the centre of each sector. We can compensate for this by
            // adding or subtracting 30 degrees from the rotor position estimate depending on the
            // sign of the velocity. This will make the rotor position estimate more accurate

            if (params_->enable_sector_position_offset_compensation) {
                if (velocity_ > 0.0f) {
                    rotor_position_ -= math::M_PI_FLOAT / 6.0f;
                } else {
                    rotor_position_ += math::M_PI_FLOAT / 6.0f;
                }

                // Implement a wraparound
                math::wraparound(rotor_position_, 0.0f, math::M_PI_FLOAT * 2.0f);
            }

            // If the velocity sign has changed, then we should reset the rotor position estimation
            // This is because the rotor position estimation is only valid for one direction of rotation
            // and prevents the rotor position estimation from being wonky at low speeds
            const bool velocity_sign_changed = (velocity_ * velocity_previous_) < 0.0f;
            const bool rotor_speed_below_minimum = ((fabs(velocity_) < params_->minimum_estimation_velocity));
            if (velocity_sign_changed || rotor_speed_below_minimum) {
                reset_estimation();
            }

            this->raw_hall_angle_ = raw_hall_angle;
            time_at_last_hall_update_ = time;
            number_of_hall_updates_++;
        }

        time_update_last_called_ = time;
    } while (false);
    return ret;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::get_rotor_position(float& rotor_position) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position = rotor_position_;

    return ret;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::get_rotor_velocity(float& rotor_velocity) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_velocity = compensated_velocity_;

    return ret;
}

app_hal_status_E ElectricalRotorPosEstimatorFromHall::get_rotor_acceleration(float& rotor_acceleration) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_acceleration = acceleration_;

    return ret;
}

app_hal_status_E SensorlessRotorFluxObserver::init(SensorlessRotorFluxObserverParams* params) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        if (params == nullptr) {
            ret = APP_HAL_ERROR;
            break;
        }

        params_ = params;

        reset_estimation();

    } while (false);
    return ret;
}

app_hal_status_E SensorlessRotorFluxObserver::update(const EstimatorInputs& inputs) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        // Clarke transform the current
        math::alpha_beta_t i_ab = math::clarke_transform(inputs.phase_current.u, inputs.phase_current.v, inputs.phase_current.w);
        // Get the y_alpha and y_beta values
        const float y_alpha = determine_flux_driving_voltage(inputs.phase_resistance, inputs.V_alpha, i_ab.alpha);
        const float y_beta = determine_flux_driving_voltage(inputs.phase_resistance, inputs.V_beta, i_ab.beta);

        // Get the eta values for the alpha and beta axes
        const float eta_alpha = determine_flux_deviation(x_alpha_, inputs.phase_inductance, i_ab.alpha);
        const float eta_beta = determine_flux_deviation(x_beta_, inputs.phase_inductance, i_ab.beta);

        // Get the estimated flux linkage squared
        const float estimated_flux_linkage_squared = eta_alpha * eta_alpha + eta_beta * eta_beta;

        // Get the pm flux squared
        const float pm_flux_squared = inputs.pm_flux_linkage * inputs.pm_flux_linkage;

        // Get the flux_dot values for the alpha and beta axes
        const float flux_dot_alpha =
            determine_flux_dot(y_alpha, params_->observer_gain, eta_alpha, estimated_flux_linkage_squared, pm_flux_squared);
        const float flux_dot_beta =
            determine_flux_dot(y_beta, params_->observer_gain, eta_beta, estimated_flux_linkage_squared, pm_flux_squared);

        // Integrate the flux_dot values to get the flux values
        const float dt = clock_.get_dt_s(inputs.time, last_run_time_);
        x_alpha_ += flux_dot_alpha * dt;
        x_beta_ += flux_dot_beta * dt;

        // Now, we should update the estimated electrical angle by using the flux values
        theta_hat_ = determine_theta_hat_from_flux_states(x_alpha_, i_ab.alpha, x_beta_, i_ab.beta, inputs.phase_inductance,
                                                          inputs.rotor_commanded_vel_sign);

        const float omega_previous_ = omega_;
        // Now, we should update the omega value
        omega_ = update_and_determine_theta_hat_dot(params_->Kp, params_->Ki, theta_hat_, dt, z1_, z2_);

        // LPF the omega value
        // First, determine tau with an fc of 10Hz
        const float tau = math::determine_tau_from_f_c(params_->vel_fc);
        // Now, LPF the omega value
        omega_ = math::low_pass_filter(omega_, omega_previous_, tau, dt);

        if ((fabs(omega_) >= params_->minimum_estimation_velocity) && (time_of_omega_exceeding_threshold_ == 0)) {
            time_of_omega_exceeding_threshold_ = inputs.time;
        }
        // If required, hysteresis clear condition can be added here
        else if ((fabs(omega_) < params_->minimum_estimation_velocity) && (time_of_omega_exceeding_threshold_ != 0)) {
            time_of_omega_exceeding_threshold_ = 0;
        } else {
            // do nothing, persist
        }

    } while (false);

    last_run_time_ = inputs.time;
    return ret;
}

float SensorlessRotorFluxObserver::determine_flux_dot(const float& flux_driving_voltage_frame, const float& observer_gain,
                                                      const float eta, const float& estimated_flux_linkage_squared,
                                                      const float& pm_flux_squared) {
    // equation 8 from the paper in same arg order.
    float x_dot = flux_driving_voltage_frame + observer_gain / 2.0f * eta * (pm_flux_squared - estimated_flux_linkage_squared);
    return x_dot;
}

float SensorlessRotorFluxObserver::determine_flux_driving_voltage(const float& phase_resistance, const float& V_frame,
                                                                  const float& i_frame) {
    // equation 4 from the paper in same arg order.
    float V_flux = V_frame - phase_resistance * i_frame;
    return V_flux;
}

float SensorlessRotorFluxObserver::determine_flux_deviation(float x_frame, float phase_inductance, float i_frame) {
    // equation 6 from the paper in same arg order.
    float eta = x_frame - phase_inductance * i_frame;
    return eta;
}

float SensorlessRotorFluxObserver::determine_theta_hat_from_flux_states(const float& x_alpha, const float& i_alpha,
                                                                        const float& x_beta, const float i_beta,
                                                                        const float& phase_inductance, const float& vel_sign) {
    IGNORE(vel_sign);
    // equation 9 from the paper in same arg order.
    const float arg1 = x_beta - phase_inductance * i_beta;
    const float arg2 = x_alpha - phase_inductance * i_alpha;
    float theta_hat = atan2f(arg1, arg2);

    // Implement a wraparound of the theta_hat value to be between 0 and 2pi
    math::wraparound(theta_hat, 0.0f, math::M_PI_FLOAT * 2.0f);

    return theta_hat;
}

float SensorlessRotorFluxObserver::update_and_determine_theta_hat_dot(const float& Kp, const float Ki, const float& theta_hat,
                                                                      const float& dt, float& z1, float& z2) {
    float theta_hat_minus_z1 = theta_hat - z1;

    // Implement a wraparound of the theta_hat_minus_z1 value to be between -pi and pi
    // This is to prevent overflow/underflow of the z1 and z2 values
    // since Z1 is roughly equal to theta hat.
    math::wraparound(theta_hat_minus_z1, -math::M_PI_FLOAT, math::M_PI_FLOAT);

    const float z1_dot = Kp * (theta_hat_minus_z1) + Ki * z2;
    const float z2_dot = theta_hat - z1;

    // Integrate the z1 and z2 values
    z1 += z1_dot * dt;
    z2 += z2_dot * dt;

    // Now, wrap around the z1 value
    math::wraparound(z1, 0.0f, math::M_PI_FLOAT * 2.0f);

    return z1_dot;  // same as omega
}

app_hal_status_E SensorlessRotorFluxObserver::reset_estimation() {
    app_hal_status_E ret = APP_HAL_OK;
    x_alpha_ = 0.0f;
    x_beta_ = 0.0f;

    last_run_time_ = clock_.get_time_us();
    theta_hat_ = 0.0f;
    omega_ = 0.0f;
    z1_ = 0.0f;
    z2_ = 0.0f;

    return ret;
}

app_hal_status_E SensorlessRotorFluxObserver::get_rotor_position(float& rotor_position) {
    app_hal_status_E ret = APP_HAL_OK;
    rotor_position = theta_hat_;
    return ret;
}

bool SensorlessRotorFluxObserver::is_estimation_valid() {
    const bool met_time_constraint =
        (time_of_omega_exceeding_threshold_ != 0) &&
        ((clock_.get_time_us() - time_of_omega_exceeding_threshold_) >= params_->minimum_vel_above_threshold_time);
    return met_time_constraint;
}

app_hal_status_E SensorlessRotorFluxObserver::get_rotor_velocity(float& rotor_velocity) {
    app_hal_status_E ret = APP_HAL_OK;
    rotor_velocity = omega_;
    return ret;
}

}  // namespace bldc_rotor_estimator
