#include "gtest/gtest.h"
#include "math.h"
#include "math_foc.hpp"
#include "math_util.hpp"

namespace math {
// Test the clarke transform function
TEST(MathFOCTest, test_clarke_transform_all_current_1) {
    // Test the clarke transform function
    math::alpha_beta_pair_t result = math::clarke_transform(1.0, 1.0, 1.0);
    EXPECT_EQ(result.alpha, 1.0);
    EXPECT_EQ(result.beta, 0.0);
}

// Test it with a value of ia = 0.5 and ib = 0.75 and ic = 0.25
TEST(MathFOCTest, test_clarke_transform_ia_0_5_ib_0_75_ic_0_25) {
    // Test the clarke transform function
    math::alpha_beta_pair_t result = math::clarke_transform(0.5, 0.75, 0.25);
    EXPECT_EQ(result.alpha, 0.5);
    // Beta result
    float beta_result = (0.75 - 0.25) / math::sqrt_3;
    EXPECT_FLOAT_EQ(result.beta, beta_result);
}

// Test the park transform function
TEST(MathFOCTest, test_park_transform_0_degrees) {
    // Test the clarke transform function
    math::dq_pair_t result = math::park_transform(2.0, 2.0, 0.0);
    EXPECT_FLOAT_EQ(result.d, 2.0);
    EXPECT_FLOAT_EQ(result.q, 2.0);
}

// Test the park transform function with 90 degrees
TEST(MathFOCTest, test_park_transform_90_degrees) {
    // Test the clarke transform function
    math::dq_pair_t result = math::park_transform(2.0, 2.0, 90.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, 2.0);
    EXPECT_FLOAT_EQ(result.q, -2.0);
}

// Test the park transform function with 180 degrees
TEST(MathFOCTest, test_park_transform_180_degrees) {
    // Test the clarke transform function
    math::dq_pair_t result = math::park_transform(2.0, 2.0, 180.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, -2.0);
    EXPECT_FLOAT_EQ(result.q, -2.0);
}

// Test the park transform function with 270 degrees
TEST(MathFOCTest, test_park_transform_270_degrees) {
    // Test the clarke transform function
    math::dq_pair_t result = math::park_transform(2.0, 2.0, 270.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, -2.0);
    EXPECT_FLOAT_EQ(result.q, 2.0);
}

// Test the park transform function with 45 degrees
TEST(MathFOCTest, test_park_transform_45_degrees) {
    // Test the clarke transform function
    math::dq_pair_t result = math::park_transform(2.0, 2.0, 45.0 * M_PI / 180.0);
    float theta_rad = 45.0 * M_PI / 180.0;
    float q_result = 2.0 * sin(theta_rad) - 2.0 * cos(theta_rad);
    float d_result = 2.0 * sin(theta_rad) + 2.0 * cos(theta_rad);
    EXPECT_FLOAT_EQ(result.d, d_result);
    EXPECT_FLOAT_EQ(result.q, q_result);
}

}  // namespace math