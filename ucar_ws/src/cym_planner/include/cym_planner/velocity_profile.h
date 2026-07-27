#ifndef CYM_PLANNER_VELOCITY_PROFILE_H_
#define CYM_PLANNER_VELOCITY_PROFILE_H_

#include <algorithm>
#include <cmath>
#include <limits>

namespace cym_planner
{

inline double approachVelocity(double current_velocity,
                               double target_velocity,
                               double maximum_rate,
                               double elapsed_seconds)
{
    const double maximum_change =
        std::max(0.0, maximum_rate) * std::max(0.0, elapsed_seconds);
    const double scale =
        std::max(1.0, std::max(std::abs(current_velocity),
                              std::abs(target_velocity)));
    const double tolerance =
        8.0 * std::numeric_limits<double>::epsilon() * scale;
    if(target_velocity > current_velocity)
    {
        if(target_velocity - current_velocity <= maximum_change + tolerance)
        {
            return target_velocity;
        }
        return current_velocity + maximum_change;
    }
    if(current_velocity - target_velocity <= maximum_change + tolerance)
    {
        return target_velocity;
    }
    return current_velocity - maximum_change;
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_VELOCITY_PROFILE_H_
