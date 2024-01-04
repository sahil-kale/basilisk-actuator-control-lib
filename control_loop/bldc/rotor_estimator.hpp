#ifndef ROTOR_ESTIMATOR_HPP
#define ROTOR_ESTIMATOR_HPP
#include "6step_util.hpp"
#include "bridge_3phase.hpp"
#include "hal_clock.hpp"
#include "math_foc.hpp"

namespace bldc_rotor_estimator {

/**
 * @brief Abstract class for a rotor position estimator that can be used by a brushless motor control loop
 */
class ElectricalRotorPosEstimator {
   public:
    ElectricalRotorPosEstimator() = default;
    virtual ~ElectricalRotorPosEstimator() = default;

    /**
     * @brief The inputs to the rotor position estimator
     * @note This is used to provide the rotor position estimator with the necessary information to estimate the rotor position
     * @note Not all information is required for all rotor position estimators
     */
    class EstimatorInputs {
       public:
        /**
         * @brief The time of the rotor position estimator update
         */
        utime_t time = 0;
        /**
         * @brief The phase voltages
         */
        hwbridge::Bridge3Phase::phase_voltage_t phase_voltage;
        /**
         * @brief The current commutation step of a trapezoidal brushless motor control loop
         * @todo This should be removed and the rotor position estimator should infer the commutation step from the phase voltages
         */
        control_loop::Bldc6Step::commutation_step_t current_commutation_step = {
            control_loop::Bldc6Step::CommutationSignal::Z_FALLING, control_loop::Bldc6Step::CommutationSignal::HIGH,
            control_loop::Bldc6Step::CommutationSignal::LOW};

        /**
         * @brief The phase currents
         */
        hwbridge::Bridge3Phase::phase_current_t phase_current;

        /**
         * @brief The applied voltage in the alpha-beta frame of the previous control loop iteration fed-forward to the rotor
         * position estimator
         */
        math::alpha_beta_t V_alpha_beta;

        /**
         * @brief The phase resistance of the motor (Ohms)
         */
        float phase_resistance = 0.0f;

        /**
         * @brief The phase inductance of the motor (H)
         */
        float phase_inductance = 0.0f;

        /**
         * @brief The permanent magnet flux linkage of the motor (Wb)
         */
        float pm_flux_linkage = 0.0f;
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

    /**
     * @brief reset the rotor position estimation
     */
    virtual app_hal_status_E reset_estimation() = 0;
};

/**
 * @brief Rotor Sector Sensor Interface
 * @note This is used to implement a hall position sensor in an abstract fashion
 */
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
     * @param angle The electrical angle of the rotor (radians), though this is discretized with 6 sectors
     * @return app_hal_status_E the status of the operation
     */
    virtual app_hal_status_E get_electrical_angle(float& angle) = 0;
};

/**
 * @brief Sensorless Rotor Sector Sensor
 * @note This sensor CANNOT be used with field oriented control as it uses back EMF to determine the rotor sector rather than by
 * using a voltage/current observer. It is intended to provide a simplistic way of using sensorless control with trapezoidal
 * control.
 */
class BldcSensorlessRotorSectorSensor : public BldcRotorSectorSensor {
   public:
    /**
     * @brief Construct a new Bldc Sensorless Rotor Sector Sensor object
     * @param bridge The bridge to use for the sensorless rotor sector sensor
     * @param clock_ The clock to use for the sensorless rotor sector sensor
     */
    BldcSensorlessRotorSectorSensor(hwbridge::Bridge3Phase& bridge, basilisk_hal::HAL_CLOCK& clock_)
        : BldcRotorSectorSensor(), bridge_(bridge), clock_(clock_) {}

    /**
     * @brief initialize the rotor sector sensor
     */
    void init();
    /**
     * @brief Reset the rotor sector sensor
     * @return app_hal_status_E the status of the reset
     */
    app_hal_status_E reset() override;

    /**
     * @brief Get the predicted electrical angle of the rotor
     * @param angle The predicted electrical angle of the rotor (radians)
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E get_electrical_angle(float& angle) override;

   protected:
    /**
     * @brief determine if a zero crossing has been detected
     * @param phase_voltage The phase voltages
     * @param current_commutation_step The current commutation step
     * @return true if a zero crossing has been detected
     */
    bool zero_crossing_detected(const hwbridge::Bridge3Phase::phase_voltage_t& phase_voltage,
                                control_loop::Bldc6Step::commutation_step_t current_commutation_step);
    /**
     * @brief Bridge to use for the sensorless rotor sector sensor
     */
    hwbridge::Bridge3Phase& bridge_;

    /**
     * @brief The estimated electrical angle of the rotor (radians)
     */
    float estimated_electrical_angle_ = 0.0f;

    /**
     * @brief The clock to use for the sensorless rotor sector sensor
     */
    basilisk_hal::HAL_CLOCK& clock_;

    /**
     * @brief The time of the last zero crossing
     */
    utime_t time_of_last_zero_crossing_ = 0;
    /**
     * @brief The time of the last commutation
     */
    utime_t time_of_last_commutation_ = 0;
};

/**
 * @brief A rotor position estimator that uses hall sensors to determine the rotor position
 */
class ElectricalRotorPosEstimatorFromHall : public ElectricalRotorPosEstimator {
   public:
    /**
     * @brief Construct a new Electrical Rotor Pos Estimator From Hall object
     * @param clock The clock to use for the rotor position estimator
     * @param sector_sensor The sector sensor to use for the rotor position estimator
     */
    ElectricalRotorPosEstimatorFromHall(basilisk_hal::HAL_CLOCK& clock,
                                        bldc_rotor_estimator::BldcRotorSectorSensor& sector_sensor)
        : clock_(clock), sector_sensor_(sector_sensor) {}

    /**
     * @brief The parameters for the rotor position estimator
     */
    class ElectricalRotorPosEstimatorFromHallParams {
       public:
        /**
         * @brief The number of hall updates before interpolation of the rotor position is permitted
         */
        uint16_t num_hall_updates_to_start;
        /**
         * @brief The maximum angle (radians) overrun of the rotor position estimator interpolator before the center of the
         * currently sensed sector of the hall sensors is returned
         */
        float max_estimate_angle_overrun;
        /**
         * @brief Whether to enable interpolation between hall updates or just use the hall angle
         * @note interpolation can provide a more accurate rotor position estimate which can improve efficency, but it can also
         * cause issues if the rotor is subjected to a large disturbance that causes a spike in acceleration or deceleration
         */
        bool enable_interpolation;
        /**
         * @brief Whether to enable sector position offset compensation
         * @note Sector offset compensation is a technique where, combined with interpolation, the rotor position estimate is
         * compensated for the offset between the center of the hall sensor sector and the actual rotor position as we can use the
         * transitions between a rotor sector and the next to determine the actual rotor position. Otherwise, the interpolator
         * will suffer from a pi/3 phase lead between the actual rotor position and the rotor position estimate
         */
        bool enable_sector_position_offset_compensation;
        /**
         * @brief The minimum velocity to use for estimation (rad/s)
         * @note This is used to prevent the rotor position estimator from using a velocity that is too low to be accurate
         */
        float minimum_estimation_velocity;
    };

    /**
     * @brief Initialize the rotor position estimator
     * @param params The rotor position estimator parameters
     * @return app_hal_status_E the status of the initialization
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    app_hal_status_E init(ElectricalRotorPosEstimatorFromHallParams* params);

    /**
     * @brief Update the rotor position estimator
     * @param inputs The EstimatorInputs to use for the rotor position estimator
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
 * @brief A sensorless rotor flux observer based on the paper here
 * https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
 */
class SensorlessRotorFluxObserver : public ElectricalRotorPosEstimator {
   public:
    /**
     * @brief Construct a new Sensorless Rotor Flux Observer object
     * @param clock The clock to use for the sensorless rotor flux observer
     */
    explicit SensorlessRotorFluxObserver(basilisk_hal::HAL_CLOCK& clock) : clock_(clock) {}

    /**
     * @brief The parameters for the sensorless rotor flux observer implementation
     */
    class SensorlessRotorFluxObserverParams {
       public:
        /**
         * @brief The observer gain that corrects for the flux deviation
         * @note This is referred to as gamma in the paper
         */
        float observer_gain;
        /**
         * @brief The minimum velocity to use for determining when the estimation is valid (rad/s)
         */
        float minimum_estimation_velocity;
        /**
         * @brief The minimum amount of time to wait before declaring the estimation valid (us)
         */
        utime_t minimum_vel_above_threshold_time;
        /**
         * @brief The proportional gain of the speed tracking controller estimate
         */
        float Kp;
        /**
         * @brief The integral gain of the speed tracking controller estimate
         */
        float Ki;

        /**
         * @brief The velocity low-pass filter cutoff frequency (Hz)
         * @note This is used to filter the velocity estimate to prevent noise from causing the velocity estimate to be too noisy,
         * but does cause phase lag
         */
        float vel_fc;
    };

    /**
     * @brief Initialize the sensorless rotor flux observer
     * @param params The sensorless rotor flux observer parameters
     * @return app_hal_status_E the status of the initialization
     * @attention The parameters are not copied, so the parameters must remain in scope for the lifetime of the control loop
     */
    app_hal_status_E init(SensorlessRotorFluxObserverParams* params);

    /**
     * @brief Update the sensorless rotor flux observer
     * @param inputs The EstimatorInputs to use for the sensorless rotor flux observer
     * @return app_hal_status_E the status of the update
     */
    app_hal_status_E update(const EstimatorInputs& inputs) override;

    /**
     * @brief Get the rotor position
     * @param rotor_position The rotor position as an electrical angle (radians)
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E get_rotor_position(float& rotor_position) override;

    /**
     * @brief Get the rotor's electrical angle velocity
     * @param rotor_velocity The rotor velocity as an electrical angular velocity (radians/s)
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E get_rotor_velocity(float& rotor_velocity) override;

    /**
     * @brief reset the rotor position estimation
     * @return app_hal_status_E the status of the operation
     */
    app_hal_status_E reset_estimation() override;

    /**
     * @brief get whether the rotor position estimation is valid
     * @return true if the rotor position estimation is valid
     * @note: the rotor position estimation is valid if the velocity is above the minimum velocity threshold for a certain amount
     * of time
     */
    bool is_estimation_valid() override;

   protected:
    /*! \cond PRIVATE */
    /**
     * @brief Determine the flux driving voltage (y) in the paper
     * @param stator_resistance The stator resistance of the motor (defined as R_s in the paper, or 3/2*R_phase)
     * @param V_frame The applied voltage in the frame of reference of the rotor (alpha or beta)
     * @param i_frame The current in the frame of reference of the rotor (alpha or beta)
     * @return The flux driving voltage (y) in the paper
     * @note Implements the equation (4) from the paper, which is really just finding
     *       the voltage on the inductor and back-emf voltage by subtracting the
     *       voltage drop across the phase resistance from the phase voltage from the
     *       V_alpha and V_beta applied voltages (can be treated like a regular DC motor in this frame)
     */
    float determine_flux_driving_voltage(const float& stator_resistance, const float& V_frame, const float& i_frame);

    /**
     * @brief Determine the flux deviation (eta)
     * @param x_frame The state variable x in the frame of reference of the rotor (alpha or beta)
     * @param stator_inductance The stator inductance of the motor (defined as L_s in the paper, or 3/2 * L_phase)
     * @param i_frame The current in the frame of reference of the rotor (alpha or beta)
     * @return The flux deviation (eta)
     * @note Implements the equation (6) from the paper by calculating the flux deviation from the permanent magnet flux linkage
     * due to the inductance
     */
    float determine_flux_deviation(float x_frame, float stator_inductance, float i_frame);

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
     * @param stator_inductance The stator inductance of the motor (defined as L_s in the paper, or 3/2 * L_phase)
     * @return The electrical angle of the rotor (theta_hat)
     */
    float determine_theta_hat_from_flux_states(const float& x_alpha, const float& i_alpha, const float& x_beta,
                                               const float i_beta, const float& stator_inductance);

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
    /** The clock to use for the sensorless rotor flux observer */
    basilisk_hal::HAL_CLOCK& clock_;

    // State variables
    /** Referred to as x1 in the paper eqn (9) */
    float x_alpha_ = 0.0f;
    /** Referred to as x2 in the paper eqn (9) */
    float x_beta_ = 0.0f;
    /** Referred to as theta_hat in the paper eqn (9) */
    float theta_hat_ = 0.0f;
    /** The last time the estimator was run */
    utime_t last_run_time_ = 0;

    /** The estimated speed variable of the estimator*/
    float omega_ = 0.0f;
    /** The time at which the omega threshold was exceeded, 0 if not exceeded */
    utime_t time_of_omega_exceeding_threshold_ = 0;
    /** Z1 state variable */
    float z1_ = 0.0f;
    /** Z2 state variable */
    float z2_ = 0.0f;

    /**
     * @brief The parameters for the sensorless rotor flux observer implementation
     */
    SensorlessRotorFluxObserverParams* params_ = nullptr;

    /*! \endcond */
};

}  // namespace bldc_rotor_estimator
#endif