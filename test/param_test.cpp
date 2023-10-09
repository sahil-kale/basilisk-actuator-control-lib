#include "gtest/gtest.h"
#include "param_service.hpp"

TEST(ParamTest, test_param_service_is_singleton) {
    // test that the pointer of 2 instances are identical
    EXPECT_EQ(&param_service::ParamServer::getInstance(), &param_service::ParamServer::getInstance());
}