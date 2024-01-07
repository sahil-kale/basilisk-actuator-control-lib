#ifndef BRIDGE_3PHASE_HPP
#define BRIDGE_3PHASE_HPP

#include "hal_common.hpp"
#include "math_util.hpp"

namespace hwbridge {

/**
 * @brief Abstract class for a 3-phase bridge that can be used to control a 3-phase inverter
 * @note This class is intended to be used as an interface for a 3-phase bridge with a complementary PWM output and a fast ADC
 * system for current sensing
 */
class Bridge3Phase {
   public:
    Bridge3Phase() = default;
    virtual ~Bridge3Phase() = default;

    /**
     * @brief Phase command typedef to hold the duty cycle and enable flag
     */
    class phase_command_t {
       public:
        phase_command_t() = default;
        /**
         * @brief Construct a phase command
         * @param duty_cycle_high_side Duty cycle for the high side PWM channel assuming complementary PWM
         * @param enable Enable the phase output. If false, the phase output should be High-Z
         * @note The duty cycle is between 0.0f and 1.0f, where 0.5 is 50% duty cycle and represents 0V (assumes complementary
         */
        phase_command_t(float duty_cycle_high_side, bool enable) : duty_cycle_high_side(duty_cycle_high_side), enable(enable) {}
        /**
         * @brief Duty cycle for the high side PWM channel assuming complementary PWM
         * @note The duty cycle is between 0.0f and 1.0f, where 0.5 is 50% duty cycle and represents 0V (assumes complementary
         * pwm)
         */
        float duty_cycle_high_side = 0.0f;
        /**
         * @brief Enable the phase output. If false, the phase output should be High-Z
         */
        bool enable = false;

        /**
         * @brief Equal operator
         * @param rhs The right hand side of the operator
         * @return bool True if the duty cycles and enable side flags are equal
         */
        bool operator==(const phase_command_t& rhs) const {
            const bool duty_cycle_high_side_equal = math::float_equals(this->duty_cycle_high_side, rhs.duty_cycle_high_side);
            const bool enable_bridge_output = this->enable == rhs.enable;
            return duty_cycle_high_side_equal && enable_bridge_output;
        }
    };

    /**
     * @brief Phase voltage readings in Volts
     */
    class phase_voltage_t {
       public:
        /**
         * @brief Phase U voltage
         */
        float u = 0.0f;

        /**
         * @brief Phase V voltage
         */
        float v = 0.0f;

        /**
         * @brief Phase W voltage
         */
        float w = 0.0f;

        /**
         * @brief Equal operator
         * @param rhs The right hand side of the operator
         * @return bool True if the phase voltages are equal
         */
        bool operator==(const phase_voltage_t& rhs) const {
            const bool u_equal = math::float_equals(this->u, rhs.u);
            const bool v_equal = math::float_equals(this->v, rhs.v);
            const bool w_equal = math::float_equals(this->w, rhs.w);
            return u_equal && v_equal && w_equal;
        }
    };

    /**
     * @brief Phase current readings in Amps
     */
    class phase_current_t {
       public:
        phase_current_t() = default;
        /**
         * @brief Construct a phase current
         * @param u Phase U current
         * @param v Phase V current
         * @param w Phase W current
         */
        phase_current_t(float u, float v, float w) : u(u), v(v), w(w) {}
        /**
         * @brief Phase U current
         */
        float u = 0.0f;
        /**
         * @brief Phase V current
         */
        float v = 0.0f;
        /**
         * @brief Phase W current
         */
        float w = 0.0f;

        /**
         * @brief Equal operator
         * @param rhs The right hand side of the operator
         * @return bool True if the phase currents are equal
         */
        bool operator==(const phase_current_t& rhs) const {
            const bool u_equal = math::float_equals(this->u, rhs.u);
            const bool v_equal = math::float_equals(this->v, rhs.v);
            const bool w_equal = math::float_equals(this->w, rhs.w);
            return u_equal && v_equal && w_equal;
        }
    };

    /**
     * @brief Initialize the 3-phase bridge
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E init() = 0;

    /**
     * @brief Set the phase voltages
     * @param u The phase U voltage
     * @param v The phase V voltage
     * @param w The phase W voltage
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E set_phase(const phase_command_t& u, const phase_command_t& v, const phase_command_t& w) = 0;

    /**
     * @brief Read the phase voltages from the bridge
     * @param phase_voltage The phase voltages in volts
     * @return app_hal_status_E The status of the operation
     */
    virtual app_hal_status_E read_phase_voltage(phase_voltage_t& phase_voltage) = 0;

    /**
     * @brief Read the phase currents from the bridge
     * @param current The phase currents in amps
     * @return app_hal_status_E The status of the operation
     * @note this function can be called from a high frequency control loop and should be implemented as efficiently as possible
     * or employ some other system to pre-trigger a current reading that can then be read from a buffer
     */
    virtual app_hal_status_E read_phase_current(phase_current_t& current) = 0;

    /**
     * @brief Read the bus voltage from the bridge
     * @param bus_voltage The bus voltage in volts
     * @return app_hal_status_E The status of the operation
     * @note this function can be called from a high frequency control loop and should be implemented in a way that does not
     * block for a long time
     */
    virtual app_hal_status_E read_bus_voltage(float& bus_voltage) = 0;

    /**
     * @brief Number of phases in the bridge
     */
    static constexpr uint8_t NUM_PHASES = 3;
};

}  // namespace hwbridge

#endif  // BRIDGE_3PHASE_HPP