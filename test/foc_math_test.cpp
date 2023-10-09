#include "gtest/gtest.h"
#include "math.h"
#include "math_foc.hpp"

namespace math {
// Test the clarke transform function
TEST(MathFOCTest, test_clarke_transform_all_current_1) {
    // Test the clarke transform function
    math::clarke_transform_result_t result = math::clarke_transform(1.0, 1.0, 1.0);
    EXPECT_EQ(result.alpha, 1.0);
    EXPECT_EQ(result.beta, 0.0);
}

// Test it with a value of ia = 0.5 and ib = 0.75 and ic = 0.25
TEST(MathFOCTest, test_clarke_transform_ia_0_5_ib_0_75_ic_0_25) {
    // Test the clarke transform function
    math::clarke_transform_result_t result = math::clarke_transform(0.5, 0.75, 0.25);
    EXPECT_EQ(result.alpha, 0.5);
    // Beta result
    float beta_result = (0.75 - 0.25) / math::sqrt_3;
    EXPECT_FLOAT_EQ(result.beta, beta_result);
}

// Test the park transform function
TEST(MathFOCTest, test_park_transform_0_degrees) {
    // Test the clarke transform function
    math::park_transform_result_t result = math::park_transform(2.0, 2.0, 0.0);
    EXPECT_FLOAT_EQ(result.d, 2.0);
    EXPECT_FLOAT_EQ(result.q, 2.0);
}

// Test the park transform function with 90 degrees
TEST(MathFOCTest, test_park_transform_90_degrees) {
    // Test the clarke transform function
    math::park_transform_result_t result = math::park_transform(2.0, 2.0, 90.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, 2.0);
    EXPECT_FLOAT_EQ(result.q, -2.0);
}

// Test the park transform function with 180 degrees
TEST(MathFOCTest, test_park_transform_180_degrees) {
    // Test the clarke transform function
    math::park_transform_result_t result = math::park_transform(2.0, 2.0, 180.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, -2.0);
    EXPECT_FLOAT_EQ(result.q, -2.0);
}

// Test the park transform function with 270 degrees
TEST(MathFOCTest, test_park_transform_270_degrees) {
    // Test the clarke transform function
    math::park_transform_result_t result = math::park_transform(2.0, 2.0, 270.0 * M_PI / 180.0);
    EXPECT_FLOAT_EQ(result.d, -2.0);
    EXPECT_FLOAT_EQ(result.q, 2.0);
}

// Test the park transform function with 45 degrees
TEST(MathFOCTest, test_park_transform_45_degrees) {
    // Test the clarke transform function
    math::park_transform_result_t result = math::park_transform(2.0, 2.0, 45.0 * M_PI / 180.0);
    float theta_rad = 45.0 * M_PI / 180.0;
    float q_result = 2.0 * sin(theta_rad) - 2.0 * cos(theta_rad);
    float d_result = 2.0 * sin(theta_rad) + 2.0 * cos(theta_rad);
    EXPECT_FLOAT_EQ(result.d, d_result);
    EXPECT_FLOAT_EQ(result.q, q_result);
}

// Test the wraparound function
TEST(MathFocTest, test_wraparound) {
    float angle = -0.4f;

    // Wrap the angle around 0 -> 1. Expect the result to be 0.6
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(angle, 0.6f);

    // Make the angle 1.2. Expect the result to be 0.2
    angle = 1.2f;
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(angle, 0.2f);

    // Make the angle 0.5. Expect the result to be 0.5
    angle = 0.5f;
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(angle, 0.5f);

    // Make the angle 0.0. Expect the result to be 0.0
    angle = 0.0f;
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(angle, 0.0f);

    // Make the angle -46.0. Expect the result to be 0.0
    angle = -46.0f;
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(angle, 0.0f);

    // Make the angle 45.3. Expect the result to be 0.3
    angle = 45.3001f;
    math::wraparound(angle, 0.0f, 1.0f);
    EXPECT_NEAR(angle, 0.3001f, 0.01);
}

// Test the trapezoidal integral function
TEST(MathFOCTest, test_trapezoidal_integral) {
    float x = 1.0f;
    float x_prev = 0.0f;
    float y = 1.0f;
    float y_prev = 0.0f;

    float result = math::trapezoidal_integral(x, x_prev, y, y_prev);
    EXPECT_FLOAT_EQ(result, 0.5f);

    // Test it now with x = 2.0f, x_prev = 1.0f, y = -1.0f, y_prev = 1.0f
    x = 2.0f;
    x_prev = 1.0f;
    y = -1.0f;
    y_prev = 1.0f;
    result = math::trapezoidal_integral(x, x_prev, y, y_prev);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

}  // namespace math