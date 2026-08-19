#include "cym_planner/inflation_recovery_schedule.h"

#include <gtest/gtest.h>

#include <limits>

TEST(InflationRecoverySchedule, ReducesByConfiguredStepUntilMinimum)
{
    EXPECT_DOUBLE_EQ(0.214, cym_planner::inflationRecoveryRadius(1));
    EXPECT_DOUBLE_EQ(0.204, cym_planner::inflationRecoveryRadius(2));
    EXPECT_DOUBLE_EQ(0.054, cym_planner::inflationRecoveryRadius(17));
    EXPECT_DOUBLE_EQ(0.05, cym_planner::inflationRecoveryRadius(18));
    EXPECT_DOUBLE_EQ(0.05, cym_planner::inflationRecoveryRadius(100));
    EXPECT_DOUBLE_EQ(
        0.20, cym_planner::nextInflationRecoveryRadius(0.21));
    EXPECT_DOUBLE_EQ(
        0.214, cym_planner::nextInflationRecoveryRadius(0.224));
    EXPECT_DOUBLE_EQ(
        0.05, cym_planner::nextInflationRecoveryRadius(0.05));
}

TEST(InflationRecoverySchedule, SupportsResolutionAlignedIndependentSteps)
{
    EXPECT_DOUBLE_EQ(
        0.204, cym_planner::nextInflationRecoveryRadius(0.224, 0.020));
    EXPECT_NEAR(
        0.218075,
        cym_planner::nextInflationRecoveryRadius(0.224, 0.005925),
        1e-12);
    EXPECT_NEAR(
        0.21215,
        cym_planner::nextInflationRecoveryRadius(0.218075, 0.005925),
        1e-12);
}

TEST(InflationRecoverySchedule, ValidatesAppliedRadius)
{
    EXPECT_TRUE(cym_planner::inflationRadiusIsValid(0.224));
    EXPECT_TRUE(cym_planner::inflationRadiusIsValid(0.05));
    EXPECT_FALSE(cym_planner::inflationRadiusIsValid(0.049));
    EXPECT_FALSE(cym_planner::inflationRadiusIsValid(-0.01));
    EXPECT_FALSE(cym_planner::inflationRadiusIsValid(
        std::numeric_limits<double>::quiet_NaN()));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
