#include "input_manager.hpp"

#include "param_service.hpp"

namespace input_manager {

InputManager::InputManager() {}

app_hal_status_E InputManager::get_board_temperature(float& temperature) {
    app_hal_status_E returnVal = APP_HAL_OK;
    if (this->hal_adc == nullptr) {
        return APP_HAL_NOT_INITIALIZED;
    }
    float voltage = 0;
    returnVal =
        this->hal_adc->read_adc(voltage, param_service::ParamServer::getInstance().compile_params.temp_sensor_adc_channel);
    if (returnVal == APP_HAL_OK) {
        temperature = (voltage - param_service::ParamServer::getInstance().compile_params.temp_sensor_volt_offset) /
                      param_service::ParamServer::getInstance().compile_params.temp_sensor_volt_per_deg;
    }
    return returnVal;
}

app_hal_status_E InputManager::get_bus_voltage(float& voltage) {
    app_hal_status_E returnVal = APP_HAL_OK;
    if (this->hal_adc == nullptr) {
        return APP_HAL_NOT_INITIALIZED;
    }
    returnVal =
        this->hal_adc->read_adc(voltage, param_service::ParamServer::getInstance().compile_params.bus_voltage_adc_channel);
    if (returnVal == APP_HAL_OK) {
        voltage = voltage *
                  (param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_1_ohms +
                   param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_2_ohms) /
                  param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_2_ohms;
    }
    return returnVal;
}

app_hal_status_E InputManager::get_motor_current(float& current) {
    app_hal_status_E returnVal = APP_HAL_OK;
    if (this->hal_adc == nullptr) {
        return APP_HAL_NOT_INITIALIZED;
    }
    float voltage = 0;
    returnVal =
        this->hal_adc->read_adc(voltage, param_service::ParamServer::getInstance().compile_params.motor_current_adc_channel);
    if (returnVal == APP_HAL_OK) {
        current = voltage / param_service::ParamServer::getInstance().compile_params.current_sense_resistor_ohms /
                  param_service::ParamServer::getInstance().compile_params.current_sense_gain;
    }
    return returnVal;
}

}  // namespace input_manager