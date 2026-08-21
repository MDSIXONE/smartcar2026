#ifndef CYM_PLANNER_PLANNER_TUNING_H_
#define CYM_PLANNER_PLANNER_TUNING_H_

#include <algorithm>
#include <cmath>

namespace cym_planner
{

const double kPlannerTuningPi = 3.14159265358979323846;

inline double finiteClamp(double value, double minimum, double maximum)
{
    if(!std::isfinite(value) ||
       !std::isfinite(minimum) ||
       !std::isfinite(maximum) ||
       minimum > maximum)
    {
        return 0.0;
    }
    return std::max(minimum, std::min(value, maximum));
}

inline double applyMinimumSpeed(double command, double minimum_speed)
{
    if(!std::isfinite(command) || !std::isfinite(minimum_speed) ||
       minimum_speed <= 0.0 || command == 0.0 ||
       std::abs(command) >= minimum_speed)
    {
        return command;
    }
    return std::copysign(minimum_speed, command);
}

struct PlannerTuning
{
    PlannerTuning()
        : linear_x_gain(0.0),
          linear_x_kd(0.0),
          angular_gain(0.0),
          angular_kd(0.0),
          max_vel_x(0.0),
          max_vel_theta(0.0),
          min_vel_x(0.06),
          min_vel_theta(0.12),
          final_yaw_gain(0.0),
          final_yaw_max_vel(0.0),
          final_yaw_tolerance(0.10),
          final_linear_x_gain(0.0),
          goal_position_tolerance(0.08),
          obstacle_lookahead_distance(0.0),
          obstacle_cost_threshold(253),
          carry_speed_scale(1.0),
          heading_slowdown_min_scale(1.0),
          command_sweep_time(0.0),
          command_sweep_step(0.025),
          approach_decel_distance(0.0),
          approach_min_vel_x(0.0),
          lateral_gain(0.0),
          max_vel_y(0.0)
    {
    }

    double linear_x_gain;
    double linear_x_kd;
    double angular_gain;
    double angular_kd;
    double max_vel_x;
    double max_vel_theta;
    double min_vel_x;
    double min_vel_theta;
    double final_yaw_gain;
    double final_yaw_max_vel;
    double final_yaw_tolerance;
    double final_linear_x_gain;
    double goal_position_tolerance;
    double obstacle_lookahead_distance;
    int obstacle_cost_threshold;
    double carry_speed_scale;
    double heading_slowdown_min_scale;
    double command_sweep_time;
    double command_sweep_step;
    double approach_decel_distance;
    double approach_min_vel_x;
    double lateral_gain;
    double max_vel_y;
};

inline const PlannerTuning& selectPlannerTuning(
    bool body_projection_enabled,
    const PlannerTuning& point_tuning,
    const PlannerTuning& body_projection_tuning)
{
    return body_projection_enabled ?
        body_projection_tuning : point_tuning;
}

inline double headingSpeedScale(
    double heading_error,
    double minimum_scale)
{
    if(!std::isfinite(heading_error) || !std::isfinite(minimum_scale))
        return 0.0;
    const double bounded_minimum =
        std::max(0.0, std::min(minimum_scale, 1.0));
    const double limited_error =
        std::min(std::abs(heading_error), kPlannerTuningPi / 2.0);
    const double cosine = std::cos(limited_error);
    return std::max(bounded_minimum, cosine * cosine);
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_PLANNER_TUNING_H_
