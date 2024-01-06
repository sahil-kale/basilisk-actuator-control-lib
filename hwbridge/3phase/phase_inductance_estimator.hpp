#ifndef PHASE_INDUCTANCE_ESTIMATOR_HPP
#define PHASE_INDUCTANCE_ESTIMATOR_HPP
#include "bridge_3phase.hpp"
#include "hal_clock.hpp"

namespace hwbridge {

/**
 * @brief Control Law for estimating the phase inductance of a 3 phase motor
 * @todo Make the axis alignment of the phase inductance estimator configurable (defaults to U)
 */
class PhaseInductanceEstimatorController {
   public:
    /// @brief Parameters for the Phase Inductance Estimator
    class Params {
       public:
        /// @brief The duration of the time the rotor phases should be grounded (us)
        utime_t brake_duration;
        /// @brief The duration of the time the rotor phases should be driven with a square wave (us)
        utime_t measurement_duration;
    };

    /**
     * @brief State of the Phase Inductance Estimator
     */
    enum class State {
        /// @brief The Phase Inductance Estimator has not been run
        NOT_STARTED,
        /// @brief Rotor dridge is commanded to ground all phases
        BRAKE_ROTOR,
        /// @brief Rotor bridge is commanded with square wave to estimate phase inductance
        MEASUREMENT_IN_PROGRESS,
        /// @brief An error has occured during the measurement
        ERROR,
        /// @brief The measurement has completed successfully
        ESTIMATE_COMPLETE
    };

    /// @brief Inputs to the Phase Inductance Estimator
    class Input {
       public:
        /// @brief The current phase currents (A)
        hwbridge::Bridge3Phase::phase_current_t phase_currents;
        /// @brief The bus voltage (V)
        float bus_voltage = 0.0f;
    };

    /// @brief Result of the Phase Inductance Estimator
    class Result {
       public:
        /// @brief The estimated phase inductance (H)
        float phase_inductance = 0.0f;
        /// @brief The state of the Phase Inductance Estimator
        PhaseInductanceEstimatorController::State state = PhaseInductanceEstimatorController::State::NOT_STARTED;
        /// @brief Whether or not the phase inductance is valid
        bool is_phase_inductance_valid = false;

        /// @brief The commands to the rotor bridge
        hwbridge::Bridge3Phase::phase_command_t phase_commands[3];
    };

    /**
     * @brief Construct a new Phase Inductance Estimator Controller object
     * @param clock Reference to the HAL_CLOCK object used to get the current time
     * @param params Parameters for the Phase Inductance Estimator
     */
    PhaseInductanceEstimatorController(basilisk_hal::HAL_CLOCK& clock, Params params) : params_(params), clock_(clock) {}

    /**
     * @brief Run the Phase Inductance Estimator
     * @param input The input to the Phase Inductance Estimator
     * @return The result of the Phase Inductance Estimator
     */
    Result run_phase_inductance_estimator(Input input);

   protected:
    /*! \cond PRIVATE */
    /**
     * @brief The desired state of the Phase Inductance Estimator
     * @param current_time The current time
     * @param input The input to the Phase Inductance Estimator
     * @return The desired state of the Phase Inductance Estimator
     */
    State get_desired_state(utime_t current_time, Input input) const;

    Params params_;
    State state_ = State::NOT_STARTED;
    basilisk_hal::HAL_CLOCK& clock_;

    utime_t brake_start_time_ = 0;
    utime_t measurement_start_time_ = 0;
    /*! \endcond */
};

}  // namespace hwbridge

#endif  // PHASE_INDUCTANCE_ESTIMATOR_HPP