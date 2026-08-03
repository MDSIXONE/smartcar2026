#ifndef CYM_PLANNER_LOCAL_ELASTIC_PATH_H_
#define CYM_PLANNER_LOCAL_ELASTIC_PATH_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace cym_planner
{

// A zero-displacement band at both ends prevents a candidate from
// teleporting sideways when it joins/leaves the global route.
inline double elasticLateralOffset(double travelled,
                                   double lookahead,
                                   double signed_peak_offset)
{
    if(!std::isfinite(travelled) || !std::isfinite(lookahead) ||
       !std::isfinite(signed_peak_offset) || lookahead <= 0.0)
    {
        return 0.0;
    }
    const double progress = std::max(
        0.0, std::min(1.0, travelled / lookahead));
    return signed_peak_offset * std::sin(3.14159265358979323846 * progress);
}

inline int elasticInterpolationSamples(double translation_distance,
                                       double translation_step,
                                       double yaw_delta,
                                       double yaw_step)
{
    if(!std::isfinite(translation_distance) || !std::isfinite(translation_step) ||
       !std::isfinite(yaw_delta) || !std::isfinite(yaw_step) ||
       translation_distance < 0.0 || translation_step <= 0.0 || yaw_step <= 0.0)
    {
        return 0;
    }
    const int translation_samples = static_cast<int>(std::ceil(
        translation_distance / translation_step));
    const int yaw_samples = static_cast<int>(std::ceil(
        std::abs(yaw_delta) / yaw_step));
    return std::max(1, std::max(translation_samples, yaw_samples));
}

inline bool elasticPlanGeometryMatches(double goal_distance,
                                       double rejoin_distance)
{
    return std::isfinite(goal_distance) && std::isfinite(rejoin_distance) &&
        goal_distance <= 0.03 && rejoin_distance <= 0.05;
}

inline bool elasticSearchTimerSurvivesEquivalentPlan(bool search_active,
                                                     bool plan_equivalent)
{
    return search_active && plan_equivalent;
}

// Candidate ranking is deliberately based on the worst footprint cell first:
// lower raw inflation cost means more clearance from every local obstacle.
// The mean cost then breaks ties, while the smallest offset is only a final
// tie-breaker so a harmless path is not needlessly deformed.
struct ElasticClearanceScore
{
    ElasticClearanceScore()
        : valid(false),
          maximum_cost(0),
          total_cost(0),
          sampled_cells(0),
          absolute_offset(0.0)
    {
    }

    bool valid;
    unsigned int maximum_cost;
    std::uint64_t total_cost;
    std::uint64_t sampled_cells;
    double absolute_offset;
};

inline bool elasticCandidateHasMoreClearance(
    const ElasticClearanceScore& candidate,
    const ElasticClearanceScore& reference)
{
    if(!candidate.valid || candidate.sampled_cells == 0 ||
       !std::isfinite(candidate.absolute_offset))
    {
        return false;
    }
    if(!reference.valid || reference.sampled_cells == 0 ||
       !std::isfinite(reference.absolute_offset))
    {
        return true;
    }
    if(candidate.maximum_cost != reference.maximum_cost)
        return candidate.maximum_cost < reference.maximum_cost;

    // Cross multiplication avoids a floating-point average and remains far
    // below uint64_t overflow for the 1 x 1 m local costmap.
    const std::uint64_t candidate_total =
        candidate.total_cost * reference.sampled_cells;
    const std::uint64_t reference_total =
        reference.total_cost * candidate.sampled_cells;
    if(candidate_total != reference_total)
        return candidate_total < reference_total;

    return candidate.absolute_offset < reference.absolute_offset;
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_LOCAL_ELASTIC_PATH_H_
