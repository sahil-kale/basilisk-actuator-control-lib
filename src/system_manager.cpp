#include "system_manager.hpp"

#include "hal_common.hpp"
#include "input_manager.hpp"
#include "param_service.hpp"

namespace system_manager {
SystemManager::SystemManager() {}

bool SystemManager::isOverTemp(float deg_c) const {
    // get compile threshold from param service
    float overtemperature_threshold_deg_c =
        param_service::ParamServer::getInstance().compile_params.overtemperature_threshold_deg_c;
    return deg_c >= overtemperature_threshold_deg_c;
}

bool SystemManager::isOverVoltage(float voltage) const {
    // get compile threshold from param service
    float overvoltage_threshold_v = param_service::ParamServer::getInstance().compile_params.max_permissable_bus_voltage;
    return voltage >= overvoltage_threshold_v;
}

bool SystemManager::isOverCurrent(float current) const {
    // get compile threshold from param service
    float overcurrent_threshold_a = param_service::ParamServer::getInstance().compile_params.max_permissable_motor_current_amps;
    return current >= overcurrent_threshold_a;
}

bool SystemManager::isEnabled() const { return controller_enabled; }

void SystemManager::enableController() { controller_enabled = true; }
void SystemManager::disableController() { controller_enabled = false; }
void SystemManager::setDesiredSpeed(float speed) { desired_speed = speed; }
void SystemManager::pollFaults() {
    // get the current temperature
    float temperature_deg_c = 0;
    app_hal_status_E status = input_manager::InputManager::getInstance().get_board_temperature(temperature_deg_c);
    if (status != APP_HAL_OK) {
        return;
    }
    // get the current voltage
    float voltage_v = 0;
    status = input_manager::InputManager::getInstance().get_bus_voltage(voltage_v);
    if (status != APP_HAL_OK) {
        return;
    }
    // get the current current
    float current_a = 0;
    status = input_manager::InputManager::getInstance().get_motor_current(current_a);
    if (status != APP_HAL_OK) {
        return;
    }
    // check if any of the faults are active
    is_fault_active = isOverTemp(temperature_deg_c) || isOverVoltage(voltage_v) || isOverCurrent(current_a);
}

SystemManager::SystemManagerState SystemManager::getDesiredState() {
    // Switch case tree to determine the desired state
    switch (state) {
        case SystemManagerState::DISABLED:
            if (is_fault_active) {
                state = SystemManagerState::FAULT;
            } else if (isEnabled()) {
                state = SystemManagerState::ENABLED;
            }
            break;
        case SystemManagerState::ENABLED:
            if (is_fault_active) {
                state = SystemManagerState::FAULT;
            } else if (!isEnabled()) {
                state = SystemManagerState::DISABLED;
            }
            break;
        case SystemManagerState::FAULT:
            if (!is_fault_active) {
                state = SystemManagerState::DISABLED;
            }
            break;
        default:
            break;
    }
    return state;
}

float SystemManager::getSpeedToRun() const { return commanded_speed; }

void SystemManager::runState(SystemManager::SystemManagerState state) {
    switch (state) {
        case SystemManagerState::DISABLED:
            commanded_speed = 0.0f;
            break;
        case SystemManagerState::ENABLED:
            commanded_speed = desired_speed;
            break;
        case SystemManagerState::FAULT:
            disableController();
            commanded_speed = 0.0f;
            break;
        default:
            break;
    }
}

void SystemManager::receive_set_speed_msg(comm::generic_message_header_t header, const void* message) {
    // Cast the message to a SetSpeedMessage
    comm::SetSpeedMessage set_speed_message;
    set_speed_message.populateDataFromBuffer((const uint8_t*)message, header.message_length);

    // Check that the channel ID is correct
    if (header.message_channel != (comm::channel_id_t)comm::ChannelID::SET_SPEED) {
        return;
    }

    // Check that the message length is correct
    if (header.message_length != sizeof(comm::SetSpeedMessage::set_speed_message_t)) {
        return;
    }

    // Set the desired speed
    setDesiredSpeed(set_speed_message.data.speed);
}

void SystemManager::receive_set_speed_msg_wrapper(comm::generic_message_header_t header, const void* message, void* context) {
    (void)context;
    SystemManager::getInstance().receive_set_speed_msg(header, message);
}

void SystemManager::run() {
    pollFaults();
    SystemManagerState desired_state = getDesiredState();
    runState(desired_state);
}

}  // namespace system_manager
