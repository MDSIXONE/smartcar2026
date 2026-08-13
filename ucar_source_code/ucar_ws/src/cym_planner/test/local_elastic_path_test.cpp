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

TEST(LocalElasticPath, ClearanceRankingPrefersSafestBandBeforeSmallestOffset)
{
    cym_planner::ElasticClearanceScore empty;
    EXPECT_FALSE(empty.valid);
    EXPECT_EQ(0u, empty.maximum_cost);

    cym_planner::ElasticClearanceScore narrow;
    narrow.valid = true;
    narrow.maximum_cost = 240;
    narrow.total_cost = 1000;
    narrow.sampled_cells = 10;
    narrow.absolute_offset = 0.02;

    cym_planner::ElasticClearanceScore clear;
    clear.valid = true;
    clear.maximum_cost = 180;
    clear.total_cost = 1500;
    clear.sampled_cells = 10;
    clear.absolute_offset = 0.08;

    EXPECT_TRUE(cym_planner::elasticCandidateHasMoreClearance(clear, narrow));
    EXPECT_FALSE(cym_planner::elasticCandidateHasMoreClearance(narrow, clear));
}

TEST(LocalElasticPath, ClearanceRankingUsesMeanCostThenSmallestOffset)
{
    cym_planner::ElasticClearanceScore lower_mean;
    lower_mean.valid = true;
    lower_mean.maximum_cost = 200;
    lower_mean.total_cost = 300;
    lower_mean.sampled_cells = 3;
    lower_mean.absolute_offset = 0.10;

    cym_planner::ElasticClearanceScore higher_mean;
    higher_mean.valid = true;
    higher_mean.maximum_cost = 200;
    higher_mean.total_cost = 800;
    higher_mean.sampled_cells = 4;
    higher_mean.absolute_offset = 0.02;

    EXPECT_TRUE(cym_planner::elasticCandidateHasMoreClearance(
        lower_mean, higher_mean));

    cym_planner::ElasticClearanceScore equal_clearance = lower_mean;
    equal_clearance.absolute_offset = 0.04;
    EXPECT_TRUE(cym_planner::elasticCandidateHasMoreClearance(
        equal_clearance, lower_mean));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
