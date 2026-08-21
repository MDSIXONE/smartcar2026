#include <gtest/gtest.h>

#include "cym_planner/planner_tuning.h"

#include <limits>

TEST(PlannerTuning, PointAndBodyModesSelectIndependentValues)
{
    cym_planner::PlannerTuning point;
    cym_planner::PlannerTuning body;
    point.linear_x_gain = 1.5;
    point.max_vel_x = 0.50;
    point.command_sweep_time = 0.0;
    body.linear_x_gain = 0.9;
    body.max_vel_x = 0.22;
    body.command_sweep_time = 0.4;

    EXPECT_DOUBLE_EQ(
        1.5, cym_planner::selectPlannerTuning(false, point, body).linear_x_gain);
    EXPECT_DOUBLE_EQ(
        0.50, cym_planner::selectPlannerTuning(false, point, body).max_vel_x);
    EXPECT_DOUBLE_EQ(
        0.9, cym_planner::selectPlannerTuning(true, point, body).linear_x_gain);
    EXPECT_DOUBLE_EQ(
        0.22, cym_planner::selectPlannerTuning(true, point, body).max_vel_x);
    EXPECT_DOUBLE_EQ(
        0.0, cym_planner::selectPlannerTuning(
            false, point, body).command_sweep_time);
    EXPECT_DOUBLE_EQ(
        0.4, cym_planner::selectPlannerTuning(
            true, point, body).command_sweep_time);
}

TEST(PlannerTuning, DestinationProfileCanTightenOnlyTheFinalApproach)
{
    cym_planner::PlannerTuning point;
    cym_planner::PlannerTuning destination;
    point.goal_position_tolerance = 0.07;
    point.final_linear_x_gain = 0.0;
    destination.goal_position_tolerance = 0.04;
    destination.final_linear_x_gain = 1.0;

    EXPECT_DOUBLE_EQ(0.07, point.goal_position_tolerance);
    EXPECT_DOUBLE_EQ(0.0, point.final_linear_x_gain);
    EXPECT_DOUBLE_EQ(0.04, destination.goal_position_tolerance);
    EXPECT_DOUBLE_EQ(1.0, destination.final_linear_x_gain);
}

TEST(PlannerTuning, BodyModeCanSlowTranslationBeforeTightTurns)
{
    EXPECT_NEAR(
        1.0, cym_planner::headingSpeedScale(0.0, 0.15), 1e-12);
    EXPECT_NEAR(
        0.25, cym_planner::headingSpeedScale(
            cym_planner::kPlannerTuningPi / 3.0, 0.15), 1e-12);
    EXPECT_NEAR(
        0.15, cym_planner::headingSpeedScale(
            cym_planner::kPlannerTuningPi / 2.0, 0.15), 1e-12);
}

TEST(PlannerTuning, PointModeCanPreserveLegacyNoSlowdown)
{
    EXPECT_DOUBLE_EQ(
        1.0, cym_planner::headingSpeedScale(
            cym_planner::kPlannerTuningPi / 2.0, 1.0));
}

TEST(PlannerTuning, NonFiniteClampFailsClosedToZero)
{
    EXPECT_DOUBLE_EQ(
        0.0, cym_planner::finiteClamp(
            std::numeric_limits<double>::quiet_NaN(), -1.0, 1.0));
    EXPECT_DOUBLE_EQ(
        0.0, cym_planner::finiteClamp(
            std::numeric_limits<double>::infinity(), -1.0, 1.0));
    EXPECT_DOUBLE_EQ(0.5, cym_planner::finiteClamp(0.5, -1.0, 1.0));
}

TEST(PlannerTuning, FloorsNonzeroCommandsAtChassisDeadZone)
{
    EXPECT_DOUBLE_EQ(
        0.06, cym_planner::applyMinimumSpeed(0.014, 0.06));
    EXPECT_DOUBLE_EQ(
        -0.06, cym_planner::applyMinimumSpeed(-0.014, 0.06));
    EXPECT_DOUBLE_EQ(
        0.06, cym_planner::applyMinimumSpeed(0.06, 0.06));
    EXPECT_DOUBLE_EQ(
        0.08, cym_planner::applyMinimumSpeed(0.08, 0.06));
    EXPECT_DOUBLE_EQ(
        0.0, cym_planner::applyMinimumSpeed(0.0, 0.06));
    EXPECT_DOUBLE_EQ(
        0.12, cym_planner::applyMinimumSpeed(0.078, 0.12));
    EXPECT_DOUBLE_EQ(
        -0.12, cym_planner::applyMinimumSpeed(-0.078, 0.12));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
