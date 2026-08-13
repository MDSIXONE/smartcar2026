#include <gtest/gtest.h>

#include "cym_planner/escape_recovery.h"

TEST(EscapeRecovery, ReplaysPointOneLeftWallContact)
{
    const cym_planner::EscapeDirection direction =
        cym_planner::selectEscapeDirection(
            -2.25, 1.25, cym_planner::kEscapePi,
            -2.485, 1.215, 0.171, 0.128);

    ASSERT_TRUE(direction.valid);
    EXPECT_EQ(cym_planner::ESCAPE_FRONT, direction.contact_side);
    EXPECT_NEAR(-1.0, direction.base_x, 1e-9);
    EXPECT_NEAR(0.0, direction.base_y, 1e-9);

    // The vehicle faces west at point 1.  Moving backward in base_link moves
    // east in map coordinates, away from the left wall.
    EXPECT_NEAR(1.0, direction.world_x, 1e-9);
    EXPECT_NEAR(0.0, direction.world_y, 1e-9);
}

TEST(EscapeRecovery, LateralContactProducesPureOppositeTranslation)
{
    const cym_planner::EscapeDirection direction =
        cym_planner::selectEscapeDirection(
            0.0, 0.0, 0.0,
            0.02, 0.14, 0.171, 0.128);

    ASSERT_TRUE(direction.valid);
    EXPECT_EQ(cym_planner::ESCAPE_LEFT, direction.contact_side);
    EXPECT_NEAR(0.0, direction.base_x, 1e-9);
    EXPECT_NEAR(-1.0, direction.base_y, 1e-9);
    EXPECT_NEAR(0.0, direction.world_x, 1e-9);
    EXPECT_NEAR(-1.0, direction.world_y, 1e-9);
}

TEST(EscapeRecovery, InvalidGeometryCannotCommandRecovery)
{
    EXPECT_FALSE(cym_planner::selectEscapeDirection(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.128).valid);
    EXPECT_FALSE(cym_planner::selectEscapeDirection(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.171, 0.0).valid);
}

TEST(EscapeRecovery, PreviewMustStayClearOrStrictlyReduceContactCells)
{
    EXPECT_TRUE(cym_planner::escapePreviewImproves(0, 0, false, false));
    EXPECT_FALSE(cym_planner::escapePreviewImproves(0, 1, true, true));
    EXPECT_FALSE(cym_planner::escapePreviewImproves(0, 0, true, false));
    EXPECT_TRUE(cym_planner::escapePreviewImproves(3, 2, true, true));
    EXPECT_TRUE(cym_planner::escapePreviewImproves(3, 0, false, false));
    EXPECT_FALSE(cym_planner::escapePreviewImproves(3, 3, true, true));
    EXPECT_FALSE(cym_planner::escapePreviewImproves(3, 0, true, false));
}

TEST(EscapeRecovery, IntermediateProjectionMayStayEqualButNeverWorsen)
{
    EXPECT_TRUE(cym_planner::escapeIntermediateDoesNotWorsen(
        1, 1, true, true));
    EXPECT_TRUE(cym_planner::escapeIntermediateDoesNotWorsen(
        1, 0, false, false));
    EXPECT_FALSE(cym_planner::escapeIntermediateDoesNotWorsen(
        1, 2, true, true));
    EXPECT_FALSE(cym_planner::escapeIntermediateDoesNotWorsen(
        1, 0, true, false));
    EXPECT_FALSE(cym_planner::escapeIntermediateDoesNotWorsen(
        0, 1, true, true));
}

TEST(EscapeRecovery, AttemptAndDistanceCapsAreBothHardLimits)
{
    EXPECT_TRUE(cym_planner::escapeBudgetAllows(0, 4, 0.00, 0.02, 0.08));
    EXPECT_TRUE(cym_planner::escapeBudgetAllows(3, 4, 0.06, 0.02, 0.08));
    EXPECT_FALSE(cym_planner::escapeBudgetAllows(4, 4, 0.08, 0.02, 0.08));
    EXPECT_FALSE(cym_planner::escapeBudgetAllows(2, 4, 0.07, 0.02, 0.08));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
