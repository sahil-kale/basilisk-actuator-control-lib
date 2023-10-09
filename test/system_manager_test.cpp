#include "system_manager.hpp"

#include "gtest/gtest.h"
#include "param_service.hpp"

namespace system_manager {

TEST(SystemManagerTest, test_system_manager_is_singleton) {
    // test that the pointer of 2 instances are identical
    EXPECT_EQ(&SystemManager::getInstance(), &SystemManager::getInstance());
}

TEST(SystemManagerTest, test_system_manager_is_overtemperature) {
    // Test that the over temp fault is triggered when the temperature is above the threshold
    // get compile threshold from param service
    float overtemperature_threshold_deg_c =
        param_service::ParamServer::getInstance().compile_params.overtemperature_threshold_deg_c;
    EXPECT_TRUE(SystemManager::getInstance().isOverTemp(overtemperature_threshold_deg_c));
    EXPECT_FALSE(SystemManager::getInstance().isOverTemp(overtemperature_threshold_deg_c - 0.1));
}

TEST(SystemManagerTest, test_system_manager_is_overvoltage) {
    // Test that the over voltage fault is triggered when the voltage is above the threshold
    // get compile threshold from param service
    float overvoltage_threshold_v = param_service::ParamServer::getInstance().compile_params.max_permissable_bus_voltage;
    EXPECT_TRUE(SystemManager::getInstance().isOverVoltage(overvoltage_threshold_v));
    EXPECT_FALSE(SystemManager::getInstance().isOverVoltage(overvoltage_threshold_v - 0.1));
}

TEST(SystemManagerTest, test_system_manager_is_overcurrent) {
    // Test that the over current fault is triggered when the current is above the threshold
    // get compile threshold from param service
    float overcurrent_threshold_a = param_service::ParamServer::getInstance().compile_params.max_permissable_motor_current_amps;
    EXPECT_TRUE(SystemManager::getInstance().isOverCurrent(overcurrent_threshold_a));
    EXPECT_FALSE(SystemManager::getInstance().isOverCurrent(overcurrent_threshold_a - 0.1));
}

// Test that the system manager state machine starts in idle
TEST(SystemManagerTest, test_system_manager_state_machine_starts_in_idle) {
    EXPECT_EQ(SystemManager::getInstance().getDesiredState(), SystemManager::SystemManagerState::DISABLED);
    // Also test that the system manager is diasabled
    EXPECT_FALSE(SystemManager::getInstance().isEnabled());
}

// Test that if the controller is enabled and no faults are active, the system manager state machine goes to ENABLED
TEST(SystemManagerTest, test_system_manager_state_machine_enabled) {
    // Confirm we are in disabled state
    EXPECT_EQ(SystemManager::getInstance().getDesiredState(), SystemManager::SystemManagerState::DISABLED);
    // set that there are no active faults
    SystemManager::getInstance().is_fault_active = false;
    // Enable the controller
    SystemManager::getInstance().enableController();
    // Check that the state machine is in ENABLED
    EXPECT_EQ(SystemManager::getInstance().getDesiredState(), SystemManager::SystemManagerState::ENABLED);
    // Also test that the system manager is enabled
    EXPECT_TRUE(SystemManager::getInstance().isEnabled());
}

// Test that a fault moves the system manager state machine to FAULT
TEST(SystemManagerTest, test_system_manager_state_machine_fault) {
    // set that there are active faults
    SystemManager::getInstance().is_fault_active = true;
    // Enable the controller
    SystemManager::getInstance().enableController();
    // Check that the state machine is in FAULT
    EXPECT_EQ(SystemManager::getInstance().getDesiredState(), SystemManager::SystemManagerState::FAULT);
}

TEST(SystemManagerTest, test_system_manager_disabled_state) {
    // Run the system manager
    SystemManager::getInstance().runState(SystemManager::SystemManagerState::DISABLED);
    // Check that the desired speed is 0
    EXPECT_EQ(SystemManager::getInstance().getSpeedToRun(), 0);

    // Run the system manager
    SystemManager::getInstance().runState(SystemManager::SystemManagerState::FAULT);
    // Check that the desired speed is 0
    EXPECT_EQ(SystemManager::getInstance().getSpeedToRun(), 0);
}

// Test that the desired speed is given when running the system manager in ENABLED
TEST(SystemManagerTest, test_system_manager_enabled_state) {
    // Set the desired speed
    SystemManager::getInstance().setDesiredSpeed(30.5);
    // Run the system manager
    SystemManager::getInstance().runState(SystemManager::SystemManagerState::ENABLED);
    // Check that the desired speed is 1
    EXPECT_EQ(SystemManager::getInstance().getSpeedToRun(), 30.5);
}

}  // namespace system_manager