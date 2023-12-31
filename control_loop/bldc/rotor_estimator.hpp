#ifndef ROTOR_ESTIMATOR_HPP
#define ROTOR_ESTIMATOR_HPP
#include "bridge_3phase.hpp"
#include "brushless_6step_commutation.hpp"
#include "hal_clock.hpp"

namespace bldc_rotor_estimator {

class ElectricalRotorPosEstimator {
   public:
    ElectricalRotorPosEstimator() = default;
    virtual ~ElectricalRotorPosEstimator() = default;

    // Define a struct for estimator inputs
    // Note that not all estimators will use all of these inputs
    // nor is it expected that all parameters are populated for all estimators
    class EstimatorInputs {
       public:
        utime_t time = 0;
        hwbridge::Bridge3Phase::phase_voltage_t phase_voltage;
        control_loop::Bldc6Step::commutation_step_t current_commutation_step = {
            control_loop::Bldc6Step::CommutationSignal::Z_FALLING, control_loop::Bldc6Step::CommutationSignal::HIGH,
            control_loop::Bldc6Step::CommutationSignal::LOW};

        hwbridge::Bridge3Phase::phase_current_t phase_current;
        float V_alpha = 0.0f;
        float V_beta = 0.0f;
        float phase_resistance = 0.0f;
        float phase_inductance = 0.0f;
        float pm_flux_linkage = 0.0f;           // Permanent magnet flux linkage (Wb)
        float rotor_commanded_vel_sign = 0.0f;  // Sign of the rotor velocity (1 or -1)
    };

    /**
     * @brief Update the rotor position estimator
     */
    virtual app_hal_status_E update(const EstimatorInputs& inputs) = 0;

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
    bool zero_crossing_detected(const hwbridge::Bridge3Phase::phase_voltage_t& phase_voltage,
                                control_loop::Bldc6Step::commutation_step_t current_commutation_step);
    hwbridge::Bridge3Phase& bridge_;
    float estimated_electrical_angle_ = 0.0f;
    basilisk_hal::HAL_CLOCK& clock_;

    utime_t time_of_last_zero_crossing_ = 0;
    utime_t time_of_last_commutation_ = 0;
};

class ElectricalRotorPosEstimatorFromHall : public ElectricalRotorPosEstimator {
   public:
    ElectricalRotorPosEstimatorFromHall(basilisk_hal::HAL_CLOCK& clock,
                                        bldc_rotor_estimator::BldcRotorSectorSensor& sector_sensor)
        : clock_(clock), sector_sensor_(sector_sensor) {}

    class ElectricalRotorPosEstimatorFromHallParams {
       public:
        uint16_t num_hall_updates_to_start;
        float max_estimate_angle_overrun;  // How much to allow the estimator to overrun the hall angle (radians)
        bool enable_interpolation;         // Whether to enable interpolation between hall updates or just use the hall angle
        bool enable_sector_position_offset_compensation;  // Whether to enable sector position offset compensation
        float minimum_estimation_velocity;                // The minimum velocity to use for estimation (rad/s)
    };

    /**
     * @brief Initialize the rotor position estimator
     * @param params The rotor position estimator parameters
     * @return app_hal_status_E the status of the initialization
     */
    app_hal_status_E init(ElectricalRotorPosEstimatorFromHallParams* params);

    /**
     * @brief Update the rotor position estimator
     * @param time The current time
     * @return app_hal_status_E the status of the update
     */
    app_hal_status_E update(const EstimatorInputs& inputs) override;

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
     * @brief Get the rotor acceleration
     * @param rotor_acceleration The rotor acceleration as an electrical angular acceleration (radians/s^2)
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E get_rotor_acceleration(float& rotor_acceleration);

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
     * @brief get whether interpolation is permitted
     * @return true if interpolation is permitted
     * @note: Factors in the num hall updates to start, max estimate angle overrun, and whether interpolation is enabled alongside
     * other factors to determine whether interpolation is permitted by the main rotor position estimator
     */
    bool is_interpolation_permitted();

    /**
     * @brief reset the rotor position estimation
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E reset_estimation() override;

   private:
    basilisk_hal::HAL_CLOCK& clock_;
    bldc_rotor_estimator::BldcRotorSectorSensor& sector_sensor_;
    float rotor_position_ = 0.0f;
    float acceleration_ = 0.0f;                         // rad/s^2
    float velocity_previous_ = 0.0f, velocity_ = 0.0f;  // rad/s
    float compensated_velocity_ = 0.0f;                 // rad/s
    float raw_hall_angle_ = 0.0f;
    utime_t time_at_last_hall_update_ = 0;
    utime_t time_update_last_called_ = 0;
    uint64_t number_of_hall_updates_ = 0;
    ElectricalRotorPosEstimatorFromHallParams* params_ = nullptr;
};

/**
 * @brief A sensorless rotor flux observer based on the paper below:
 * @link https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
 */
class SensorlessRotorFluxObserver : public ElectricalRotorPosEstimator {
   public:
    explicit SensorlessRotorFluxObserver(basilisk_hal::HAL_CLOCK& clock) : clock_(clock) {}

    class SensorlessRotorFluxObserverParams {
       public:
        float observer_gain;                       // Referred to as gamma in the paper eqn, the observer gain, rad/s
        float minimum_estimation_velocity;         // The minimum velocity to use for estimation (rad/s)
        utime_t minimum_vel_above_threshold_time;  // The time to wait before declaring the estimation valid (us)
        float Kp;                                  // The proportional gain of the speed tracking controller estimate
        float Ki;                                  // The integral gain of the speed tracking controller estimate

        float vel_fc;  // The velocity low-pass filter cutoff frequency (rad/s)
    };

    app_hal_status_E init(SensorlessRotorFluxObserverParams* params);

    app_hal_status_E update(const EstimatorInputs& inputs) override;

    app_hal_status_E get_rotor_position(float& rotor_position) override;

    app_hal_status_E get_rotor_velocity(float& rotor_velocity) override;

    app_hal_status_E reset_estimation() override;

    bool is_estimation_valid() override;

   protected:
    /**
     * @brief Determine the flux driving voltage (y) in the paper
     * @param phase_resistance The phase resistance of the motor
     * @param V_frame The applied voltage in the frame of reference of the rotor (alpha or beta)
     * @param i_frame The current in the frame of reference of the rotor (alpha or beta)
     * @return The flux driving voltage (y) in the paper
     * @note Implements the equation (4) from the paper, which is really just finding
     *       the voltage on the inductor and back-emf voltage by subtracting the
     *       voltage drop across the phase resistance from the phase voltage from the
     *       V_alpha and V_beta applied voltages (can be treated like a regular DC motor in this frame)
     */
    float determine_flux_driving_voltage(const float& phase_resistance, const float& V_frame, const float& i_frame);

    /**
     * @brief Determine the flux deviation (eta)
     * @param x_frame The state variable x in the frame of reference of the rotor (alpha or beta)
     * @param phase_inductance The phase inductance of the motor
     * @param i_frame The current in the frame of reference of the rotor (alpha or beta)
     * @return The flux deviation (eta)
     * @note Implements the equation (6) from the paper by calculating the flux deviation from the permanent magnet flux linkage
     * due to the inductance
     */
    float determine_flux_deviation(float x_frame, float phase_inductance, float i_frame);

    /**
     * @brief Determine the state variable delta (x_dot)
     * @param flux_driving_voltage_frame The flux driving voltage (y) in the paper
     * @param observer_gain The observer gain (gamma)
     * @param eta The flux deviation (eta)
     * @param estimated_flux_linkage_squared The estimated flux linkage squared (eta^2)
     * @param pm_flux_squared The permanent magnet flux linkage squared (psi^2)
     * @return The state variable delta (x_dot)
     * @note Implements the equation (6 and 8) from the paper
     * @note Determines the flux dynamics (x_dot) based on the current state of the system
     */
    float determine_flux_dot(const float& flux_driving_voltage_frame, const float& observer_gain, const float eta,
                             const float& estimated_flux_linkage_squared, const float& pm_flux_squared);

    /**
     * @brief Determine the electrical angle of the rotor (theta_hat) from the flux states
     * @param x_alpha The state variable x_alpha
     * @param i_alpha The current in the alpha frame
     * @param x_beta The state variable x_beta
     * @param i_beta The current in the beta frame
     * @param phase_inductance The phase inductance of the motor
     * @param vel_sign The sign of the rotor velocity (1 or -1)
     * @return The electrical angle of the rotor (theta_hat)
     */
    float determine_theta_hat_from_flux_states(const float& x_alpha, const float& i_alpha, const float& x_beta,
                                               const float i_beta, const float& phase_inductance, const float& vel_sign);

    /**
     * @brief Determine the velocity of the rotor (theta_hat_dot) using equations 11-13 from the paper
     * @param Kp The proportional gain of the speed trackiner controller estimate
     * @param Ki The integral gain of the speed trackiner controller estimate
     * @param theta_hat The electrical angle of the rotor (theta_hat)
     * @param dt The time since the last update
     * @param z1 The state variable z1 (note: this is passed by reference and updated in this function)
     * @param z2 The state variable z2 (note: this is passed by reference and updated in this function)
     */
    float update_and_determine_theta_hat_dot(const float& Kp, const float Ki, const float& theta_hat, const float& dt, float& z1,
                                             float& z2);

    basilisk_hal::HAL_CLOCK& clock_;

    // State variables
    float x_alpha_ = 0.0f;    // Referred to as x1 in the paper eqn (9)
    float x_beta_ = 0.0f;     // Referred to as x2 in the paper eqn (9)
    float theta_hat_ = 0.0f;  // Referred to as theta_hat in the paper eqn (9)
    utime_t last_run_time_ = 0;

    float omega_ = 0.0f;
    utime_t time_of_omega_exceeding_threshold_ = 0;  // The time at which the omega threshold was exceeded, 0 if not exceeded
    float z1_ = 0.0f;
    float z2_ = 0.0f;

    SensorlessRotorFluxObserverParams* params_ = nullptr;
};

}  // namespace bldc_rotor_estimator
#endif