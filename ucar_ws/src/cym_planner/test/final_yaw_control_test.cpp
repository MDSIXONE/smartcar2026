#include <gtest/gtest.h>

#include "cym_planner/final_yaw_control.h"

namespace
{
const double kPi = cym_planner::kFinalYawPi;
}

TEST(FinalYawControl, KeepsRotationDirectionAcrossPiBranchCut)
{
    cym_planner::FinalYawTracker tracker;

    EXPECT_NEAR(tracker.update(kPi - 0.02), kPi - 0.02, 1e-9);

    // A tiny pose update can make tf::getYaw() report the other side of the
    // branch cut.  The controller must not reverse its angular command.
    const double continuous_error = tracker.update(-kPi + 0.02);
    EXPECT_GT(continuous_error, 0.0);
    EXPECT_NEAR(continuous_error, kPi + 0.02, 1e-9);
}

TEST(FinalYawControl, PreservesClockwiseDirectionAcrossPiBranchCut)
{
    cym_planner::FinalYawTracker tracker;

    EXPECT_NEAR(tracker.update(-kPi + 0.02), -kPi + 0.02, 1e-9);

    const double continuous_error = tracker.update(kPi - 0.02);
    EXPECT_LT(continuous_error, 0.0);
    EXPECT_NEAR(continuous_error, -kPi - 0.02, 1e-9);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
