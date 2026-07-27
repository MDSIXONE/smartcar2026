#include <gtest/gtest.h>

#include "cym_planner/velocity_profile.h"

TEST(VelocityProfile, LinearVelocityReachesZeroWithoutACommandStep)
{
    double velocity = 0.40;
    for(int cycle = 0; cycle < 8; ++cycle)
    {
        velocity = cym_planner::approachVelocity(
            velocity, 0.0, 1.0, 0.05);
    }

    EXPECT_DOUBLE_EQ(0.0, velocity);
}

TEST(VelocityProfile, RateLimitAppliesInBothDirections)
{
    EXPECT_DOUBLE_EQ(
        0.35, cym_planner::approachVelocity(0.40, 0.0, 1.0, 0.05));
    EXPECT_DOUBLE_EQ(
        -0.35, cym_planner::approachVelocity(-0.40, 0.0, 1.0, 0.05));
    EXPECT_DOUBLE_EQ(
        0.15, cym_planner::approachVelocity(0.10, 0.30, 1.0, 0.05));
}

TEST(VelocityProfile, InvalidRateOrPeriodCannotChangeVelocity)
{
    EXPECT_DOUBLE_EQ(
        0.40, cym_planner::approachVelocity(0.40, 0.0, 0.0, 0.05));
    EXPECT_DOUBLE_EQ(
        0.40, cym_planner::approachVelocity(0.40, 0.0, 1.0, 0.0));
}

TEST(VelocityProfile, LowSpeedBlockedCommandStopsWithoutAZeroStep)
{
    double velocity = 0.08;
    double previous_velocity = velocity;
    for(int cycle = 0; cycle < 7; ++cycle)
    {
        velocity = cym_planner::approachVelocity(
            velocity, 0.0, 0.25, 0.05);
        EXPECT_LE(previous_velocity - velocity, 0.0125 + 1e-12);
        EXPECT_LE(velocity, previous_velocity);
        previous_velocity = velocity;
    }

    EXPECT_DOUBLE_EQ(0.0, velocity);
}

TEST(VelocityProfile, RolloutCommandChangesAreBoundedPerControlCycle)
{
    EXPECT_DOUBLE_EQ(
        0.025, cym_planner::approachVelocity(0.0, 0.40, 0.50, 0.05));
    EXPECT_DOUBLE_EQ(
        2.20, cym_planner::approachVelocity(2.40, 0.20, 4.0, 0.05));
    EXPECT_DOUBLE_EQ(
        -2.20, cym_planner::approachVelocity(-2.40, -0.20, 4.0, 0.05));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
