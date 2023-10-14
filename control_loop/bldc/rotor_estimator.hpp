#ifndef ROTOR_ESTIMATOR_HPP
#define ROTOR_ESTIMATOR_HPP
#include "bridge_3phase.hpp"
#include "brushless_6step_commutation.hpp"
#include "hal_clock.hpp"

namespace bldc_rotor_estimator {

class BldcElectricalRotorPositionEstimator {
   public:
    BldcElectricalRotorPositionEstimator() = default;
    virtual ~BldcElectricalRotorPositionEstimator() = default;

    /**
     * @brief Update the rotor position estimator
     */
    virtual app_hal_status_E update(utime_t time) = 0;

    /**
     * @brief Get the rotor position
     * @param rotor_position The rotor position as an electrical angle (radians)
     */
    virtual app_hal_status_E get_rotor_position(float& rotor_position) = 0;

    /**
     * @brief Get the rotor velocity
     * @param rotor_velocity The rotor velocity as an electrical angular velocity (radians/s)
     */
    virtual app_hal_status_E get_rotor_velocity(float& rotor_velocity) = 0;

    /**
     * @brief get whether the rotor position estimation is valid
     *
     * @return true if the rotor position estimation is valid
     */
    virtual bool is_estimation_valid() = 0;

    virtual app_hal_status_E reset_estimation() = 0;
};

// Define a generic class for a sensor that returns the sector of the rotor
class BldcRotorSectorSensor {
   public:
    BldcRotorSectorSensor() = default;
    virtual ~BldcRotorSectorSensor() = default;

    /**
     * @brief Reset the rotor sector sensor
     * @return app_hal_status_E the status of the reset
     */
    virtual app_hal_status_E reset() = 0;

    /**
     * @brief Get the sector of the rotor
     * @param sector The electrical angle of the rotor (radians), though this is discretized with 6 sectors
     * @return app_hal_status_E the status of the operation
     */
    virtual app_hal_status_E get_electrical_angle(float& angle) = 0;
};

class BldcSensorlessRotorSectorSensor : public BldcRotorSectorSensor {
   public:
    BldcSensorlessRotorSectorSensor(hwbridge::Bridge3Phase& bridge, basilisk_hal::HAL_CLOCK& clock_)
        : BldcRotorSectorSensor(), bridge_(bridge), clock_(clock_) {}

    /**
     * @brief initialize the rotor sector sensor
     */
    void init();
    app_hal_status_E reset() override;
    app_hal_status_E get_electrical_angle(float& angle) override;

   protected:
    bool zero_crossing_detected(const hwbridge::Bridge3Phase::bemf_voltage_t& bemf_voltage,
                                control_loop::Bldc6StepCommutationTypes::commutation_step_t current_commutation_step);
    hwbridge::Bridge3Phase& bridge_;
    float estimated_electrical_angle_ = 0.0f;
    basilisk_hal::HAL_CLOCK& clock_;

    utime_t time_of_last_zero_crossing_ = 0;
    utime_t time_of_last_commutation_ = 0;
};

class BldcElectricalRotorPositionEstimatorFromHall : public BldcElectricalRotorPositionEstimator {
   public:
    BldcElectricalRotorPositionEstimatorFromHall(basilisk_hal::HAL_CLOCK& clock,
                                                 bldc_rotor_estimator::BldcRotorSectorSensor& sector_sensor)
        : clock_(clock), sector_sensor_(sector_sensor) {}

    class BldcElectricalRotorPositionEstimatorFromHallParams {
       public:
        uint16_t num_hall_updates_to_start;
        float max_estimate_angle_overrun;  // How much to allow the estimator to overrun the hall angle (radians)
        bool enable_interpolation;         // Whether to enable interpolation between hall updates or just use the hall angle
    };

    /**
     * @brief Initialize the rotor position estimator
     * @param params The rotor position estimator parameters
     * @return app_hal_status_E the status of the initialization
     */
    app_hal_status_E init(BldcElectricalRotorPositionEstimatorFromHallParams* params);

    /**
     * @brief Update the rotor position estimator
     * @param time The current time
     * @return app_hal_status_E the status of the update
     */
    app_hal_status_E update(utime_t time) override;

    /**
     * @brief Get the rotor position
     * @param rotor_position The rotor position as an electrical angle (radians)
     * @return app_hal_status_E the status of the operation
     * @note: rotor_position is from 0 -> 2*pi
     */
    app_hal_status_E get_rotor_position(float& rotor_position) override;

    /**
     * @brief Get the rotor velocity
     * @param rotor_velocity The rotor velocity as an electrical angular velocity (radians/s)
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E get_rotor_velocity(float& rotor_velocity) override;

    /**
     * @brief Get the raw hall angle
     * @param raw_hall_angle The raw hall angle (radians)
     * @return app_hal_status_E the status of the operation
     * @note: raw_hall_angle is from 0 -> 2*pi
     */
    app_hal_status_E get_raw_hall_angle(float& raw_hall_angle) {
        raw_hall_angle = raw_hall_angle_;
        return APP_HAL_OK;
    }

    /**
     * @brief get whether the rotor position estimation is valid
     * @return true if the rotor position estimation is valid
     * @note: the rotor position estimation is valid if the number of hall updates is greater than the number of hall updates to
     * start
     */
    bool is_estimation_valid() override;

    /**
     * @brief reset the rotor position estimation
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E reset_estimation() override;

    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::BldcRotorSectorSensor& sector_sensor_;
    float rotor_position_ = 0.0f;
    float velocity_ = 0.0f;              // rad/s
    float compensated_velocity_ = 0.0f;  // rad/s
    float raw_hall_angle_ = 0.0f;
    utime_t time_at_last_hall_update_ = 0;
    utime_t time_update_last_called_ = 0;
    uint64_t number_of_hall_updates_ = 0;
    BldcElectricalRotorPositionEstimatorFromHallParams* params_ = nullptr;
};

}  // namespace bldc_rotor_estimator
#endif