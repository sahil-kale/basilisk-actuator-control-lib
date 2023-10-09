#ifndef PARAM_SERVER_HPP
#define PARAM_SERVER_HPP
#include <stdint.h>

#include "hal_common.hpp"

namespace param_service {

class ParamServer {
   public:
    static ParamServer& getInstance() {
        static ParamServer instance;
        return instance;
    }

    struct comm_message_ids {
        const uint16_t arm_disarm_msg = 0x0001;
        const uint16_t set_target_speed_msg = 0x0002;
    };

    // Compile time parameters
    struct compile_params {
        const bool enable_usb_interface = false;
        const bool enable_can_interface = false;
        const bool enable_temperature_sense_monitoring = true;
        const float overtemperature_threshold_deg_c = 100;
        // Analog values
        const float analog_reference_voltage = 3.3;
        const uint8_t analog_resolution_bits = 10;
        const uint8_t temp_sensor_adc_channel = 0;
        const uint8_t bus_voltage_adc_channel = 1;
        const uint8_t motor_current_adc_channel = 2;
        // Thermistor parameters (TODO: make these reflect reality)
        const float temp_sensor_volt_offset = 1;
        const float temp_sensor_volt_per_deg = 1;
        // Bus voltage parameters
        const float max_permissable_bus_voltage = 40;
        const float bus_voltage_dividor_resistor_1_ohms = 1000;
        const float bus_voltage_dividor_resistor_2_ohms = 1000;
        // Current sense resistor value for PCBA
        const float max_permissable_motor_current_amps = 60;
        const float current_sense_resistor_ohms = 0.1;
        const float current_sense_gain = 100;
        // Control Loop Parameters
        const float control_loop_period_s = 0.001;
        const float control_loop_deadband = 1;
        const float control_loop_max_output = 100;
        const float control_loop_min_output = -100;
        // H Bridge parameters
        const float h_bridge_dead_time_us = 10;
        const bool h_bridge_brake_mode_enabled = false;
        // Stepper motor parameters
        const float stepper_motor_max_speed_steps_per_s = 1000;
        const float stepper_motor_max_current_amps = 1.5;
        const bool stepper_motor_simple_switcher_enabled = true;
        const bool stepper_motor_disable_current_pid =
            true;  // TODO: remove this. this will use a liner map from the current scales to the pwm duty cycle
        const float stepper_motor_current_to_pwm_duty_cycle_slope = 50;

        // 3 phase motor parameters
        const utime_t sensorless_phase_commutation_step_time_us = 4000;
        const utime_t sensorless_phase_motor_startup_sequence_time_us = sensorless_phase_commutation_step_time_us * 100;
        const float sensorless_speed_deadband_scale = 0.3f;  // 10% of max speed, lower and we cannot detect back emf
        const float sensorless_startup_speed = 1.0f;
        const float sensored_speed_deadband_scale = 0.1f;  // 10% of max speed, lower and actuating the motor does not make sense
        const bool sensorless_bemf_enable_backemf_skip_overrun = false;
        const bool log_zero_crossing_in_sensored_mode = false;
        const utime_t bemf_zero_crossing_timeout_us = 1875;  // time at which we do not detect a ZC after a commutation step

        // FOC parameters
        const uint8_t num_hall_updates_to_start = 10;
        const utime_t foc_start_timeout_period = 1000000;  // 1 second
        const float open_loop_velocity_rad_s = 2000.0f;

        // Observer parameters
        const float observer_angle_error_kp = 10.0f;
    };

    compile_params compile_params;

    // Get message ids
    // Done this way to allow for dynamic message ids
    comm_message_ids get_message_ids() { return message_ids; }

   private:
    ParamServer();
    comm_message_ids message_ids;
};

}  // namespace param_service

#endif
