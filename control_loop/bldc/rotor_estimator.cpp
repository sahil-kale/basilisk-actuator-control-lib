#include "rotor_estimator.hpp"

#include "brushless_6step_commutation.hpp"
#include "math.h"
#include "math_util.hpp"

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
        hwbridge::Bridge3Phase::bemf_voltage_t bemf_voltage;
        ret = bridge_.read_bemf(bemf_voltage);
        const utime_t current_time = clock_.get_time_us();

        if (ret != APP_HAL_OK) {
            break;
        }

        // Get the commutation step as a function of the electrical angle
        control_loop::Bldc6StepCommutationTypes::commutation_step_t step =
            control_loop::Bldc6StepCommutationTypes::determine_commutation_step_from_theta(estimated_electrical_angle_);

        // Check if a zero crossing has occurred
        const bool zero_crossing = zero_crossing_detected(bemf_voltage, step);

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
    const hwbridge::Bridge3Phase::bemf_voltage_t& bemf_voltage,
    control_loop::Bldc6StepCommutationTypes::commutation_step_t current_commutation_step) {
    float phase_sum = 0.0f;
    control_loop::Bldc6StepCommutationTypes::CommutationSignal zero_crossing_signal =
        control_loop::Bldc6StepCommutationTypes::CommutationSignal::Z_RISING;
    float undriven_phase_voltage = 0.0f;

    const float bemf_voltages[hwbridge::Bridge3Phase::NUM_PHASES] = {bemf_voltage.u, bemf_voltage.v, bemf_voltage.w};

    for (uint8_t i = 0; i < hwbridge::Bridge3Phase::NUM_PHASES; i++) {
        if ((current_commutation_step.signals[i] != control_loop::Bldc6StepCommutationTypes::CommutationSignal::Z_FALLING) &&
            (current_commutation_step.signals[i] != control_loop::Bldc6StepCommutationTypes::CommutationSignal::Z_RISING)) {
            phase_sum += bemf_voltages[i];
        } else {
            zero_crossing_signal = current_commutation_step.signals[i];
            undriven_phase_voltage = bemf_voltages[i];
        }
    }

    float zero_crossing_threshold = phase_sum / 2.0f;  // NOTE: This requires the bemf voltage to run when the PWM is ON
    bool return_value = false;
    if (zero_crossing_signal == control_loop::Bldc6StepCommutationTypes::CommutationSignal::Z_RISING) {
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

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::reset_estimation() {
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

bool BldcElectricalRotorPositionEstimatorFromHall::is_estimation_valid() {
    return true;  // Since we know the absolute position, we should always return true
}

bool BldcElectricalRotorPositionEstimatorFromHall::is_interpolation_permitted() {
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

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::init(BldcElectricalRotorPositionEstimatorFromHallParams* params) {
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

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::update(utime_t time) {
    app_hal_status_E ret = APP_HAL_OK;
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

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::get_rotor_position(float& rotor_position) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position = rotor_position_;

    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::get_rotor_velocity(float& rotor_velocity) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_velocity = compensated_velocity_;

    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::get_rotor_acceleration(float& rotor_acceleration) {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_acceleration = acceleration_;

    return ret;
}

}  // namespace bldc_rotor_estimator
