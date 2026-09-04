#include <gtest/gtest.h>

#include "pi_franka_hand.hpp"

namespace {

constexpr double kMaxWidth = 0.08079;  // A Franka Hand's measured stroke.

}  // namespace

TEST(FrankaHandCalibration, PublicPositionIsZeroClosedOneOpen) {
    EXPECT_FLOAT_EQ(FrankaHandTransport::width_to_position(0.0, kMaxWidth), 0.0f);
    EXPECT_FLOAT_EQ(FrankaHandTransport::width_to_position(kMaxWidth, kMaxWidth), 1.0f);
    EXPECT_NEAR(FrankaHandTransport::width_to_position(kMaxWidth / 2.0, kMaxWidth), 0.5f, 1e-6f);
    EXPECT_DOUBLE_EQ(FrankaHandTransport::position_to_width(0.0f, kMaxWidth), 0.0);
    EXPECT_DOUBLE_EQ(FrankaHandTransport::position_to_width(1.0f, kMaxWidth), kMaxWidth);
}

TEST(FrankaHandCalibration, WidthAndPositionRoundTrip) {
    for (const double width : {0.0, 0.01, 0.035, 0.0605, kMaxWidth}) {
        const float position = FrankaHandTransport::width_to_position(width, kMaxWidth);
        EXPECT_NEAR(FrankaHandTransport::position_to_width(position, kMaxWidth), width, 1e-6);
    }
}

TEST(FrankaHandCalibration, OutOfRangeInputsSaturateInsteadOfWrapping) {
    // The hand reports a width slightly past the stroke it advertises, and a
    // caller may ask for more than fully open; both clamp.
    EXPECT_FLOAT_EQ(FrankaHandTransport::width_to_position(kMaxWidth + 0.01, kMaxWidth), 1.0f);
    EXPECT_FLOAT_EQ(FrankaHandTransport::width_to_position(-0.001, kMaxWidth), 0.0f);
    EXPECT_DOUBLE_EQ(FrankaHandTransport::position_to_width(1.5f, kMaxWidth), kMaxWidth);
    EXPECT_DOUBLE_EQ(FrankaHandTransport::position_to_width(-0.5f, kMaxWidth), 0.0);
}

TEST(FrankaHandCalibration, AnUnreadMaximumWidthReportsClosedRatherThanDividingByZero) {
    // max_width is 0 until the first state packet arrives.
    EXPECT_FLOAT_EQ(FrankaHandTransport::width_to_position(0.04, 0.0), 0.0f);
    EXPECT_DOUBLE_EQ(FrankaHandTransport::position_to_width(1.0f, 0.0), 0.0);
}
