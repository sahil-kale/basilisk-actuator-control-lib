#include "math_util.hpp"

#include "gtest/gtest.h"
#include "math.h"

namespace math {

// Test the wraparound function
TEST(MathUtilTest, test_wraparound) {
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
TEST(MathUtilTest, test_trapezoidal_integral) {
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