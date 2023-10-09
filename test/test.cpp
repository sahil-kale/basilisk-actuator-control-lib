#include "example.hpp"
#include "gtest/gtest.h"

TEST(ExampleTest, test_add) { EXPECT_EQ(2, test_add_func(1, 1)); }