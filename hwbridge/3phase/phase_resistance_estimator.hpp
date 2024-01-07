#ifndef PHASE_RESISTANCE_ESTIMATOR_HPP
#define PHASE_RESISTANCE_ESTIMATOR_HPP
#include "bridge_3phase.hpp"
#include "pid.hpp"

namespace hwbridge {
/**
 * @brief Class to estimate the phase resistance of a 3-phase motor
 * @details This class will estimate the phase resistance of a 3-phase motor by pushing a predefined current through the bridge
 * and measuring the voltage drop to estimate the resistance.
 * @todo Make the resistance estimation more robust by averaging the voltage drop and current over the measurement period
 * @todo Make the estimation alignment axis configurable (currently it is fixed to the u-axis)
 */
class PhaseResistanceEstimatorController {
   public:
    /// @brief Parameters for the Phase Resistance Estimator
    class Params {
       public:
        /// @brief The duration of the brake phase (us)
        utime_t brake_duration = 0;
        /// @brief The duration of the measurement phase (us)
        utime_t measurement_duration = 0;
        /// @brief The target current to use for the measurement phase (A)
        float target_current = 0.0f;

        /// @brief The margin within which the measured current and target current must have reached to use for the phase
        /// resistance measurement (A)
        float current_tolerance = 0.1f;
        /**
         * @brief The maximum voltage to use for the measurement phase (V)
         * @note If this is set to 0, the voltage will not be limited
         */
        float max_voltage = 0.0f;

        /// @brief Kp for the current controller
        float current_kp = 0.0f;
        /// @brief Ki for the current controller
        float current_ki = 0.0f;
    };

    /**
     * @brief State of the Phase Resistance Estimator
     */
    enum class State {
        /// @brief The Phase Resistance Estimator has not been run
        NOT_STARTED,
        /// @brief Rotor dridge is commanded to ground all phases
        BRAKE_ROTOR,
        /// @brief Rotor bridge is commanded with a predefined test current to estimate phase resistance
        MEASUREMENT_IN_PROGRESS,
        /// @brief An error has occured during the measurement
        ERROR,
        /// @brief The measurement has completed successfully
        ESTIMATE_COMPLETE
    };

    /// @brief Inputs to the Phase Resistance Estimator
    class Input {
       public:
        /// @brief The current phase currents (A)
        hwbridge::Bridge3Phase::phase_current_t phase_currents;
        /// @brief The bus voltage (V)
        float bus_voltage = 0.0f;
    };

    /// @brief Result of the Phase Resistance Estimator
    class Result {
       public:
        /// @brief The estimated phase resistance (Ohm)
        float phase_resistance = 0.0f;
        /// @brief The state of the Phase Resistance Estimator
        PhaseResistanceEstimatorController::State state = PhaseResistanceEstimatorController::State::NOT_STARTED;
        /// @brief Whether or not the phase resistance is valid
        bool is_phase_resistance_valid = false;

        /// @brief The commands to the rotor bridge
        hwbridge::Bridge3Phase::phase_command_t phase_commands[3];
    };

    /**
     * @brief Construct a new Phase Resistance Estimator Controller object
     * @param clock Reference to the HAL_CLOCK object used to get the current time
     * @param params Parameters for the Phase Resistance Estimator
     */
    PhaseResistanceEstimatorController(basilisk_hal::HAL_CLOCK& clock, const Params& params)
        : params_(params),
          clock_(clock),
          current_controller_(params_.current_kp, params_.current_ki, 0.0f, 0.0f, params_.max_voltage, 0.0f, clock_) {}

    /**
     * @brief Initialize (or reset) the Phase Resistance Estimator
     * @param params Parameters for the Phase Resistance Estimator
     */
    void init(const Params& params);

    /**
     * @brief Run the Phase Resistance Estimator
     * @param input The inputs to the Phase Resistance Estimator
     * @return The result of the Phase Resistance Estimator
     */
    Result run_phase_resistance_estimator(Input input);

   protected:
    /*! \cond PRIVATE */
    /**
     * @brief The desired state of the Phase Resistance Estimator
     * @param current_time The current time
     * @param input The inputs to the Phase Resistance Estimator
     * @return The desired state of the Phase Resistance Estimator
     */
    State desired_state(utime_t current_time, Input input) const;

    Params params_;
    State state_ = State::NOT_STARTED;
    basilisk_hal::HAL_CLOCK& clock_;
    pid::PID<float> current_controller_;

    utime_t brake_start_time_ = 0;
    utime_t measurement_start_time_ = 0;
    float commanded_voltage_ = 0.0f;

    /* \endcond */
};

}  // namespace hwbridge

#endif  // PHASE_RESISTANCE_ESTIMATOR_HPP