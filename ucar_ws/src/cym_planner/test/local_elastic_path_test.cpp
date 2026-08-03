#include <gtest/gtest.h>

#include <limits>

#include "cym_planner/local_elastic_path.h"

TEST(LocalElasticPath, BandStartsAndEndsOnGlobalPlan)
{
    EXPECT_DOUBLE_EQ(0.0, cym_planner::elasticLateralOffset(0.0, 0.60, 0.08));
    EXPECT_NEAR(0.08,
                cym_planner::elasticLateralOffset(0.30, 0.60, 0.08),
                1e-12);
    EXPECT_NEAR(0.0,
                cym_planner::elasticLateralOffset(0.60, 0.60, 0.08),
                1e-12);
}

TEST(LocalElasticPath, InvalidInputsFailClosedToNoOffset)
{
    EXPECT_DOUBLE_EQ(0.0, cym_planner::elasticLateralOffset(0.2, 0.0, 0.08));
    EXPECT_DOUBLE_EQ(0.0, cym_planner::elasticLateralOffset(
                              0.2, 0.6,
                              std::numeric_limits<double>::quiet_NaN()));
}

TEST(LocalElasticPath, RotationRequiresAdditionalFootprintSamples)
{
    EXPECT_EQ(1, cym_planner::elasticInterpolationSamples(0.005, 0.015, 0.0, 0.05));
    EXPECT_EQ(4, cym_planner::elasticInterpolationSamples(0.005, 0.015, 0.20, 0.05));
    EXPECT_EQ(0, cym_planner::elasticInterpolationSamples(0.1, 0.0, 0.0, 0.05));
}

TEST(LocalElasticPath, EquivalentGlobalPlanPreservesElasticBand)
{
    EXPECT_TRUE(cym_planner::elasticPlanGeometryMatches(0.01, 0.04));
    EXPECT_FALSE(cym_planner::elasticPlanGeometryMatches(0.04, 0.04));
    EXPECT_FALSE(cym_planner::elasticPlanGeometryMatches(0.01, 0.06));
}

TEST(LocalElasticPath, EquivalentPlanDoesNotResetPendingSearchTimeout)
{
    EXPECT_TRUE(cym_planner::elasticSearchTimerSurvivesEquivalentPlan(true, true));
    EXPECT_FALSE(cym_planner::elasticSearchTimerSurvivesEquivalentPlan(true, false));
    EXPECT_FALSE(cym_planner::elasticSearchTimerSurvivesEquivalentPlan(false, true));
}
