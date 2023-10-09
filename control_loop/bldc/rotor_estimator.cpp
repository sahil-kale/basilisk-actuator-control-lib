#include "rotor_estimator.hpp"

#include "math.h"
#include "math_util.hpp"

namespace bldc_rotor_estimator {

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::reset_estimation() {
    app_hal_status_E ret = APP_HAL_OK;

    rotor_position_ = 0.0f;
    velocity_ = 0.0f;
    compensated_velocity_ = 0.0f;
    raw_hall_angle_ = 0.0f;
    time_at_last_hall_update_ = 0;
    time_update_last_called_ = 0;
    number_of_hall_updates_ = 0;

    // Update the rotor position with the sector sensor
    uint8_t sector = 0;
    ret = sector_sensor_.get_sector(sector);
    if (ret == APP_HAL_OK) {
        rotor_position_ = sector * 2.0 * math::M_PI_FLOAT / 6.0f;
    }

    return ret;
}

bool BldcElectricalRotorPositionEstimatorFromHall::is_estimation_valid() {
    bool ret = false;
    if (params_ != nullptr) {
        bool num_hall_updates_to_start = (number_of_hall_updates_ > params_->num_hall_updates_to_start);
        // Also make the return conditional if the position estimate no greater than the param for tolerance
        bool rotor_position_tolerance = (fabs(rotor_position_ - raw_hall_angle_) < params_->max_estimate_angle_overrun);
        ret = num_hall_updates_to_start && rotor_position_tolerance;
    }
    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::init(
    BldcElectricalRotorPositionEstimatorFromHallParams_t* params) {
    app_hal_status_E ret = APP_HAL_OK;

    // Set the internal params pointer
    params_ = params;

    return ret;
}

app_hal_status_E BldcElectricalRotorPositionEstimatorFromHall::update(utime_t time) {
    app_hal_status_E ret = APP_HAL_OK;
    do {
        if (params_ == nullptr) {
            ret = APP_HAL_ERROR;
            break;
        }

        uint8_t sector = 0;

        ret = sector_sensor_.get_sector(sector);

        if (ret != APP_HAL_OK) {
            break;
        }

        float raw_hall_angle = sector * 2 * M_PI / 6;
        // We should not update velocity estimations if the raw hall angle is the same as the previous one
        if (fabs(raw_hall_angle - raw_hall_angle_) > 0.001f) {
            // Calculate velocity with a simple differentiation
            const float time_delta_since_hall_update =
                (float)(time - time_at_last_hall_update_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;

            float raw_hall_angle_diff = raw_hall_angle - raw_hall_angle_;

            // Account for over/underflow
            // NOTE: if the rotor delta theta is greater than pi radians, then this detection will not work
            math::wraparound(raw_hall_angle_diff, -math::M_PI_FLOAT, math::M_PI_FLOAT);

            velocity_ = raw_hall_angle_diff / time_delta_since_hall_update;

            // Calculate a compensated velocity to account for position error and smoothly compensate for it
            compensated_velocity_ = velocity_;
            // TODO: implement compensated_velocity_ = velocity_ * (1 - ((rotor_position_ - raw_hall_angle_diff) /
            // raw_hall_angle_diff));

            rotor_position_ = raw_hall_angle;
            this->raw_hall_angle_ = raw_hall_angle;
            time_at_last_hall_update_ = time;
            number_of_hall_updates_++;
        }

        if (params_->enable_interpolation) {
            const float current_measurement_period =
                (float)(time - time_update_last_called_) / basilisk_hal::HAL_CLOCK::kMicrosecondsPerSecond;
            // Update the rotor position with the velocity estimate
            rotor_position_ += compensated_velocity_ * current_measurement_period;
            // Implement a wraparound
            math::wraparound(rotor_position_, 0.0f, float(2.0f * M_PI));
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
}  // namespace bldc_rotor_estimator
