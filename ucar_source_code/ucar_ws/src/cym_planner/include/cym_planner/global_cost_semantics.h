#ifndef CYM_PLANNER_GLOBAL_COST_SEMANTICS_H_
#define CYM_PLANNER_GLOBAL_COST_SEMANTICS_H_

#include <cstdint>

namespace cym_planner
{

// nav_msgs/OccupancyGrid is the public representation of Costmap2D:
// raw 253 (inscribed inflation) is published as 99, raw 254 (lethal) as 100,
// and raw 255 (unknown) as -1.
inline bool globalCellBlocksProjectedFootprint(std::int8_t occupancy)
{
    return occupancy == 100 || occupancy < 0;
}

// Costmap2D raw 253 is inflation around the robot's inscribed radius.  The
// complete physical footprint is already projected here, so counting 253
// would inflate it twice.  Raw 254 is lethal and 255 is unknown.
inline bool localCellBlocksProjectedFootprint(std::uint8_t cost)
{
    return cost >= 254;
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_GLOBAL_COST_SEMANTICS_H_
