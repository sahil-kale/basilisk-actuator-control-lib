#include "control_loop.hpp"
#include "gtest/gtest.h"

namespace control_loop {

enum class TestWarnings { WARN_1, WARN_2, TOTAL_WARNING_COUNT };

enum class TestErrors { ERROR_1, ERROR_2, TOTAL_ERROR_COUNT };

TEST(ControlLoopStatusTest, test_reset) {
    ControlLoop::ControlLoopStatus<TestErrors, TestWarnings> status;
    status.reset();
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::OK);

    // Make sure has_error returns false for all errors
    for (uint8_t i = 0; i < status.kNumErrors; i++) {
        EXPECT_FALSE(status.has_error(static_cast<TestErrors>(i)));
    }

    // Make sure has_warning returns false for all warnings
    for (uint8_t i = 0; i < status.kNumWarnings; i++) {
        EXPECT_FALSE(status.has_warning(static_cast<TestWarnings>(i)));
    }
}

// Check Error toggling
TEST(ControlLoopStatusTest, test_set_error) {
    ControlLoop::ControlLoopStatus<TestErrors, TestWarnings> status;
    status.reset();

    // Set error 1 to true
    status.set_error(TestErrors::ERROR_1, true);
    EXPECT_TRUE(status.has_error(TestErrors::ERROR_1));
    EXPECT_FALSE(status.has_error(TestErrors::ERROR_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::ERROR);

    // Set error 2 to true
    status.set_error(TestErrors::ERROR_2, true);
    EXPECT_TRUE(status.has_error(TestErrors::ERROR_1));
    EXPECT_TRUE(status.has_error(TestErrors::ERROR_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::ERROR);

    // Set error 1 to false
    status.set_error(TestErrors::ERROR_1, false);
    EXPECT_FALSE(status.has_error(TestErrors::ERROR_1));
    EXPECT_TRUE(status.has_error(TestErrors::ERROR_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::ERROR);

    // Set error 2 to false
    status.set_error(TestErrors::ERROR_2, false);
    EXPECT_FALSE(status.has_error(TestErrors::ERROR_1));
    EXPECT_FALSE(status.has_error(TestErrors::ERROR_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::OK);
}

// Check warning toggling
TEST(ControlLoopStatusTest, test_set_warning) {
    ControlLoop::ControlLoopStatus<TestErrors, TestWarnings> status;
    status.reset();

    // Set warning 1 to true
    status.set_warning(TestWarnings::WARN_1, true);
    EXPECT_TRUE(status.has_warning(TestWarnings::WARN_1));
    EXPECT_FALSE(status.has_warning(TestWarnings::WARN_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::WARNING);

    // Set warning 2 to true
    status.set_warning(TestWarnings::WARN_2, true);
    EXPECT_TRUE(status.has_warning(TestWarnings::WARN_1));
    EXPECT_TRUE(status.has_warning(TestWarnings::WARN_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::WARNING);

    // Set warning 1 to false
    status.set_warning(TestWarnings::WARN_1, false);
    EXPECT_FALSE(status.has_warning(TestWarnings::WARN_1));
    EXPECT_TRUE(status.has_warning(TestWarnings::WARN_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::WARNING);

    // Set warning 2 to false
    status.set_warning(TestWarnings::WARN_2, false);
    EXPECT_FALSE(status.has_warning(TestWarnings::WARN_1));
    EXPECT_FALSE(status.has_warning(TestWarnings::WARN_2));
    EXPECT_EQ(status.status, ControlLoop::ControlLoopBaseStatus::OK);
}

}  // namespace control_loop