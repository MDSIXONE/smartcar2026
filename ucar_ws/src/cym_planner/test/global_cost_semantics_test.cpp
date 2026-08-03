#include <gtest/gtest.h>

#include "cym_planner/global_cost_semantics.h"

TEST(GlobalCostSemantics, InscribedInflationDoesNotMeanPhysicalContact)
{
    // Costmap2D raw 253 is published as OccupancyGrid value 99.
    EXPECT_FALSE(cym_planner::globalCellBlocksProjectedFootprint(99));
}

TEST(GlobalCostSemantics, LethalObstacleMeansPhysicalContact)
{
    // Costmap2D raw 254 is published as OccupancyGrid value 100.
    EXPECT_TRUE(cym_planner::globalCellBlocksProjectedFootprint(100));
}

TEST(GlobalCostSemantics, UnknownSpaceRemainsSafetyBlocked)
{
    EXPECT_TRUE(cym_planner::globalCellBlocksProjectedFootprint(-1));
}

TEST(LocalCostSemantics, InscribedInflationDoesNotMeanPhysicalContact)
{
    EXPECT_FALSE(cym_planner::localCellBlocksProjectedFootprint(253));
}

TEST(LocalCostSemantics, LethalObstacleMeansPhysicalContact)
{
    EXPECT_TRUE(cym_planner::localCellBlocksProjectedFootprint(254));
}

TEST(LocalCostSemantics, UnknownSpaceRemainsSafetyBlocked)
{
    EXPECT_TRUE(cym_planner::localCellBlocksProjectedFootprint(255));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
