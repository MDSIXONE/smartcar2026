#ifndef CYM_PLANNER_INFLATION_RECOVERY_SCHEDULE_H_
#define CYM_PLANNER_INFLATION_RECOVERY_SCHEDULE_H_

#include <algorithm>
#include <cmath>

namespace cym_planner
{

const double kDefaultInflationRadius = 0.224;
const double kInflationRecoveryStep = 0.01;
const double kMinimumInflationRadius = 0.05;

inline double inflationRecoveryRadius(
    int completed_reductions,
    double start_radius = kDefaultInflationRadius,
    double reduction_step = kInflationRecoveryStep,
    double minimum_radius = kMinimumInflationRadius)
{
    return std::max(
        minimum_radius,
        start_radius - static_cast<double>(completed_reductions) *
            reduction_step);
}

inline double nextInflationRecoveryRadius(
    double current_radius,
    double reduction_step = kInflationRecoveryStep,
    double minimum_radius = kMinimumInflationRadius)
{
    return inflationRecoveryRadius(
        1, current_radius, reduction_step, minimum_radius);
}

inline bool inflationRadiusIsValid(
    double radius,
    double minimum_radius = kMinimumInflationRadius)
{
    return std::isfinite(radius) &&
        std::isfinite(minimum_radius) &&
        minimum_radius >= 0.0 &&
        radius >= minimum_radius;
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_INFLATION_RECOVERY_SCHEDULE_H_
