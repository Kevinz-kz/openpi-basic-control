#include <gtest/gtest.h>

#include "pi_franka_hand.hpp"

// These construct a transport, so they live in the target that links
// libfranka; the calibration helpers are header-inline and tested in
// test_franka_hand.cpp instead.

TEST(FrankaHandLifecycle, StartingWithoutAnAddressFails) {
    FrankaHandTransport transport{FrankaHandConfig{}};
    EXPECT_FALSE(transport.start());
    const EffectorTransportState state = transport.effector_state();
    EXPECT_FALSE(state.connected);
    EXPECT_FALSE(state.activated);
    EXPECT_FALSE(transport.has_effector_fault());
}

TEST(FrankaHandLifecycle, CommandsBeforeStartAreDroppedInsteadOfCrashing) {
    // No threads are running and no gripper exists, so a stray command or a
    // teardown has to be a no-op rather than a use of a null connection.
    FrankaHandTransport transport{FrankaHandConfig{}};
    transport.set_target(0.0f, 1.0f, 1.0f);
    transport.hold();
    transport.stop();
    EXPECT_FALSE(transport.effector_state().connected);
}

TEST(FrankaHandLifecycle, ActivatingBeforeStartFails) {
    FrankaHandTransport transport{FrankaHandConfig{}};
    EXPECT_FALSE(transport.activate());
}
