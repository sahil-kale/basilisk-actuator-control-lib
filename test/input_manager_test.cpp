#include "input_manager.hpp"

#include "gtest/gtest.h"
#include "mock_hal_adc.hpp"
#include "param_service.hpp"

namespace input_manager {
using namespace ::testing;
// Test that the input manager does not crash if the hal_adc is not registered
TEST(InputManagerTest, test_get_board_temperature_no_hal_adc) {
    float returned_temperature = 0;
    EXPECT_EQ(APP_HAL_NOT_INITIALIZED, InputManager::getInstance().get_board_temperature(returned_temperature));
}

TEST(InputManagerTest, test_get_board_temperature) {
    // Create a mock hal_adc object
    basilisk_hal::MOCK_HAL_ADC hal_adc;
    // Register the mock hal_adc object with the input manager
    InputManager::getInstance().register_hal_adc(&hal_adc);
    float test_temperature = 0;
    // Expect a call to the HAL_ADC read_adc function. Return that the app hall status is ok
    // The channel call should match the compile parameter temp_sensor_adc_channel
    EXPECT_CALL(hal_adc, read_adc(_, param_service::ParamServer::getInstance().compile_params.temp_sensor_adc_channel))
        .WillOnce(Return(APP_HAL_OK));
    EXPECT_EQ(APP_HAL_OK, InputManager::getInstance().get_board_temperature(test_temperature));
}

// Test the bus voltage function in which the bus voltage is returned correctly. Inject 3.0V into the ADC and expect 6.0V to be
// returned as a function of the compile parameters
TEST(InputManagerTest, test_get_bus_voltage) {
    // Create a mock hal_adc object
    basilisk_hal::MOCK_HAL_ADC hal_adc;
    // Register the mock hal_adc object with the input manager
    InputManager::getInstance().register_hal_adc(&hal_adc);
    float test_voltage = 3.0;
    float actual_voltage = test_voltage;
    // Expect the test_voltage to be a function of the 2 resistors that make up the voltage divider
    actual_voltage *= (param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_1_ohms +
                       param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_2_ohms) /
                      param_service::ParamServer::getInstance().compile_params.bus_voltage_dividor_resistor_2_ohms;
    // Expect a call to the HAL_ADC read_adc function. Return that the app hall status is ok
    // The channel call should match the compile parameter bus_voltage_adc_channel
    EXPECT_CALL(hal_adc, read_adc(_, param_service::ParamServer::getInstance().compile_params.bus_voltage_adc_channel))
        .WillOnce(DoAll(SetArgReferee<0>(test_voltage), Return(APP_HAL_OK)));
    EXPECT_EQ(APP_HAL_OK, InputManager::getInstance().get_bus_voltage(test_voltage));
    EXPECT_FLOAT_EQ(test_voltage, actual_voltage);
}

// Test the current function in which the current is returned correctly. Inject 1V into the ADC and expect the correct voltage
// given the compile params
TEST(InputManagerTest, test_get_current) {
    // Create a mock hal_adc object
    basilisk_hal::MOCK_HAL_ADC hal_adc;
    // Register the mock hal_adc object with the input manager
    InputManager::getInstance().register_hal_adc(&hal_adc);
    float test_voltage = 1;
    float actual_current = test_voltage / param_service::ParamServer::getInstance().compile_params.current_sense_resistor_ohms /
                           param_service::ParamServer::getInstance().compile_params.current_sense_gain;
    // Expect a call to the HAL_ADC read_adc function. Return that the app hall status is ok
    // The channel call should match the compile parameter current_adc_channel
    EXPECT_CALL(hal_adc, read_adc(_, param_service::ParamServer::getInstance().compile_params.motor_current_adc_channel))
        .WillOnce(DoAll(SetArgReferee<0>(test_voltage), Return(APP_HAL_OK)));
    EXPECT_EQ(APP_HAL_OK, InputManager::getInstance().get_motor_current(test_voltage));
    EXPECT_FLOAT_EQ(test_voltage, actual_current);
}

}  // namespace input_manager
