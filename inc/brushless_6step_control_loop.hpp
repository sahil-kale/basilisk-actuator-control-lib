#ifndef BRUSHLESS_6STEP_CONTROL_LOOP_HPP
#define BRUSHLESS_6STEP_CONTROL_LOOP_HPP

// forward declare when in unit test
#if defined(UNIT_TEST) || defined(DEBUG)
class Brushless6StepControlLoopTest;
#endif

#include <stdlib.h>

#include "bridge_3phase.hpp"
#include "control_loop.hpp"
#include "hal_clock.hpp"
namespace control_loop {

// Define a brushless 6step control loop class that inherits from ControlLoop
class Brushless6StepControlLoop : public ControlLoop {
   public:
    enum class Brushless6StepControlLoopState {
        STOP,
        START,  // Sensorless startup state
        RUN,    // Sensorless run state
    };

    enum class CommutationSignal {
        HIGH,
        LOW,
        Z_RISING,
        Z_FALLING,
    };

    typedef union {
        CommutationSignal signals[hwbridge::Bridge3Phase::NUM_PHASES];
        struct {
            CommutationSignal u;
            CommutationSignal v;
            CommutationSignal w;
        };
    } commutation_step_t;

    static constexpr uint8_t num_commutation_steps = 6;
    const commutation_step_t commutation_steps[num_commutation_steps] = {
        {CommutationSignal::Z_FALLING, CommutationSignal::HIGH, CommutationSignal::LOW},
        {CommutationSignal::LOW, CommutationSignal::HIGH, CommutationSignal::Z_RISING},
        {CommutationSignal::LOW, CommutationSignal::Z_FALLING, CommutationSignal::HIGH},
        {CommutationSignal::Z_RISING, CommutationSignal::LOW, CommutationSignal::HIGH},
        {CommutationSignal::HIGH, CommutationSignal::LOW, CommutationSignal::Z_FALLING},
        {CommutationSignal::HIGH, CommutationSignal::Z_RISING, CommutationSignal::LOW},
    };

    class Brushless6StepControlLoopParams {
       public:
        float sensored_speed_deadband_scale;
        float sensorless_speed_deadband_scale;
        float sensorless_phase_motor_startup_sequence_time_us;
        float sensorless_startup_speed;
        float sensorless_phase_commutation_step_time_us;
        bool log_zero_crossing_in_sensored_mode;
        bool sensorless_bemf_enable_backemf_skip_overrun;
        float bemf_zero_crossing_timeout_us;
    };

    void init(Brushless6StepControlLoopParams* params);

    Brushless6StepControlLoop(hwbridge::Bridge3Phase& motor, basilisk_hal::HAL_CLOCK& clock,
                              hwbridge::BldcRotorSectorSensor* rotor_sensor = nullptr)
        : motor_(motor), clock_(clock) {
        rotor_sensor_ = rotor_sensor;
    }

    void run(float speed) override;

   protected:
    hwbridge::Bridge3Phase& motor_;
    Brushless6StepControlLoopParams* params_ = nullptr;
    basilisk_hal::HAL_CLOCK& clock_;
    hwbridge::BldcRotorSectorSensor* rotor_sensor_;
    Brushless6StepControlLoopState state_ = Brushless6StepControlLoopState::STOP;
    utime_t time_at_start_ = 0;
    utime_t zero_crossing_time_ = 0;
    utime_t last_commutation_step_switch_time_ = 0;
    uint8_t commutation_step_ = 0;

    // Make an array of delta times for each commutation step to store experimentally
    // measured delta times
    static constexpr uint8_t num_commutation_step_delta_samples = 100;
    utime_t commutation_step_delta_times_[num_commutation_step_delta_samples] = {0};
    // Scale factor that determines a 'safety factor' for the commutation step delta times
    // If we expect a commutation step to take 1000us, but it takes 1000us * scale_factor, then we will consider it a zero
    // crossing
    float commutation_step_delta_time_scale_factor_ = 2.0f;
    // Variable to store the average commutation step delta time
    utime_t average_commutation_step_delta_time_ = 0;

    float motor_speed_ = 0.0f;

    /**
     * @brief Get the desired state of the control loop
     */
    Brushless6StepControlLoopState get_desired_state(utime_t current_time_us, utime_t time_at_start, float motor_speed);

    /**
     * @brief Detect 0 crossing of the motor
     */
    bool zero_crossing_detected(const hwbridge::Bridge3Phase::bemf_voltage_t& bemf_voltage, uint8_t commutation_step);

    /**
     * @brief Generate the duty cycles for the 3 phases based on the commutation step
     * @param phase_command array of phase commands to be filled in, u-v-w
     * @param commutation_step the commutation step to generate the duty cycles for
     * @param motor_speed the motor speed to generate the duty cycles for. 0 - 1.0f
     */
    void generate_commutation_duty_cycles(hwbridge::Bridge3Phase::phase_command_t phase_command[3], uint8_t commutation_step,
                                          float motor_speed);

    /**
     * @brief Switch the commutation step
     */
    void commutation_step_switch_sensorless(utime_t current_time_us);

    void update_average_commutation_step_delta_time(utime_t current_time_us) {
        average_commutation_step_delta_time_ =
            calculate_average_commutation_step_delta_time(current_time_us - last_commutation_step_switch_time_,
                                                          commutation_step_delta_times_, num_commutation_step_delta_samples);
        last_commutation_step_switch_time_ = current_time_us;
    }

    // Make a function that determines the average commutation step delta time
    // based on the commutation_step_delta_times_ array
    /**
     * @brief Calculate the average commutation step delta time
     *
     * @param new_commutation_step_delta_time the new commutation step delta time to add to the array
     * @param average_commutation_step_delta_time_array the array of commutation step delta times
     * @param size the size of the array
     *
     * @return utime_t the average commutation step delta time. NOTE: 0 if the array is not full
     */
    utime_t calculate_average_commutation_step_delta_time(utime_t new_commutation_step_delta_time,
                                                          utime_t* average_commutation_step_delta_time_array, size_t size);

    void reset_commutation_step_delta_times() {
        for (size_t i = 0; i < num_commutation_step_delta_samples; i++) {
            commutation_step_delta_times_[i] = 0;
        }
        average_commutation_step_delta_time_ = 0;
    }

#if defined(UNIT_TEST) || defined(DEBUG)
    friend class Brushless6StepControlLoopTest;
#endif
};
}  // namespace control_loop

#endif  // BRUSHLESS_6STEP_CONTROL_LOOP_HPP
