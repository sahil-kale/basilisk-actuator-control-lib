#ifndef SYSTEM_MANAGER_HPP
#define SYSTEM_MANAGER_HPP
#include "comm_manager.hpp"

namespace system_manager {

class SystemManager {
   public:
    static SystemManager& getInstance() {
        static SystemManager instance;
        return instance;
    }

    // Enum containing system manager states
    enum class SystemManagerState {
        DISABLED,
        ENABLED,
        FAULT,
    };

    /**
     * @brief Run the system manager
     */
    void run();

    /**
     * @brief Run the specific system manager state
     */
    void runState(SystemManagerState state);

    /**
     * @brief Get wheter the controller is enabled or not
     */
    bool isEnabled() const;

    /**
     * @brief Enable the controller
     * TODO: Add flag that permits sending speed as allowing this to enable
     */

    void enableController();
    /**
     * @brief Disable the controller
     */
    void disableController();

    void setDesiredSpeed(float speed);
    /**
     * @brief Check if the system is over temperature
     * @param deg_c The temperature in degrees celsius
     * @return true if the system is over temperature
     */
    bool isOverTemp(float deg_c) const;

    /**
     * @brief Check if the system is over voltage
     * @param voltage The voltage in volts
     * @return true if the system is over voltage
     */
    bool isOverVoltage(float voltage) const;

    /**
     * @brief Check if the system is over current
     * @param current The current in amps
     * @return true if the system is over current
     */
    bool isOverCurrent(float current) const;

    /**
     * @brief Poll the system for faults
     */
    void pollFaults();

    bool is_fault_active{false};  // Whether the system has an active fault or not

    // Get the desired state of the system manager
    SystemManagerState getDesiredState();

    /**
     * @brief Get the speed to run the motor at for control loop
     * @return The speed to run the motor at. -1.0f -> 1.0f
     */
    float getSpeedToRun() const;

    static void receive_set_speed_msg_wrapper(comm::generic_message_header_t header, const void* message, void* context);

   private:
    SystemManager();
    SystemManagerState state{SystemManager::SystemManagerState::DISABLED};
    bool controller_enabled{false};  // Controls whether the controller is enabled or not
    float desired_speed{0.0f};       // The desired speed to run the motor at
    float commanded_speed{0.0f};     // The speed to run the motor at

    void receive_set_speed_msg(comm::generic_message_header_t header, const void* message);
};
}  // namespace system_manager

#endif