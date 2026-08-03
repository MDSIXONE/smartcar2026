#ifndef CYM_PLANNER_ESCAPE_RECOVERY_H_
#define CYM_PLANNER_ESCAPE_RECOVERY_H_

#include <cmath>

namespace cym_planner
{

const double kEscapePi = 3.14159265358979323846;

enum EscapeContactSide
{
    ESCAPE_NONE = 0,
    ESCAPE_FRONT,
    ESCAPE_REAR,
    ESCAPE_LEFT,
    ESCAPE_RIGHT
};

struct EscapeDirection
{
    EscapeDirection()
        : valid(false),
          contact_side(ESCAPE_NONE),
          base_x(0.0),
          base_y(0.0),
          world_x(0.0),
          world_y(0.0)
    {
    }

    bool valid;
    EscapeContactSide contact_side;
    double base_x;
    double base_y;
    double world_x;
    double world_y;
};

inline EscapeDirection selectEscapeDirection(
    double robot_world_x,
    double robot_world_y,
    double robot_world_yaw,
    double contact_world_x,
    double contact_world_y,
    double footprint_half_length,
    double footprint_half_width)
{
    EscapeDirection result;
    if(!std::isfinite(robot_world_x) ||
       !std::isfinite(robot_world_y) ||
       !std::isfinite(robot_world_yaw) ||
       !std::isfinite(contact_world_x) ||
       !std::isfinite(contact_world_y) ||
       !std::isfinite(footprint_half_length) ||
       !std::isfinite(footprint_half_width) ||
       footprint_half_length <= 0.0 ||
       footprint_half_width <= 0.0)
    {
        return result;
    }

    const double cosine = std::cos(robot_world_yaw);
    const double sine = std::sin(robot_world_yaw);
    const double delta_world_x = contact_world_x - robot_world_x;
    const double delta_world_y = contact_world_y - robot_world_y;
    const double contact_base_x =
        cosine * delta_world_x + sine * delta_world_y;
    const double contact_base_y =
        -sine * delta_world_x + cosine * delta_world_y;
    if(std::hypot(contact_base_x, contact_base_y) < 1e-9)
        return result;

    const double longitudinal_score =
        std::abs(contact_base_x) / footprint_half_length;
    const double lateral_score =
        std::abs(contact_base_y) / footprint_half_width;
    if(longitudinal_score >= lateral_score)
    {
        result.base_x = contact_base_x >= 0.0 ? -1.0 : 1.0;
        result.contact_side =
            contact_base_x >= 0.0 ? ESCAPE_FRONT : ESCAPE_REAR;
    }
    else
    {
        result.base_y = contact_base_y >= 0.0 ? -1.0 : 1.0;
        result.contact_side =
            contact_base_y >= 0.0 ? ESCAPE_LEFT : ESCAPE_RIGHT;
    }

    result.world_x = cosine * result.base_x - sine * result.base_y;
    result.world_y = sine * result.base_x + cosine * result.base_y;
    result.valid = true;
    return result;
}

inline bool escapePreviewImproves(
    unsigned int current_contact_count,
    unsigned int preview_contact_count,
    bool preview_blocked,
    bool preview_has_recoverable_contacts)
{
    if(current_contact_count == 0)
        return !preview_blocked;
    if(preview_blocked && !preview_has_recoverable_contacts)
        return false;
    return preview_contact_count < current_contact_count;
}

inline bool escapeIntermediateDoesNotWorsen(
    unsigned int current_contact_count,
    unsigned int candidate_contact_count,
    bool candidate_blocked,
    bool candidate_has_recoverable_contacts)
{
    if(current_contact_count == 0)
        return !candidate_blocked;
    if(candidate_blocked && !candidate_has_recoverable_contacts)
        return false;
    return candidate_contact_count <= current_contact_count;
}

inline bool escapeBudgetAllows(
    int completed_attempts,
    int maximum_attempts,
    double completed_distance,
    double step_distance,
    double maximum_total_distance)
{
    if(completed_attempts < 0 ||
       maximum_attempts <= 0 ||
       !std::isfinite(completed_distance) ||
       !std::isfinite(step_distance) ||
       !std::isfinite(maximum_total_distance) ||
       completed_distance < 0.0 ||
       step_distance <= 0.0 ||
       maximum_total_distance <= 0.0)
    {
        return false;
    }
    return completed_attempts < maximum_attempts &&
        completed_distance + step_distance <=
            maximum_total_distance + 1e-9;
}

inline const char* escapeContactSideName(EscapeContactSide side)
{
    switch(side)
    {
    case ESCAPE_FRONT:
        return "front";
    case ESCAPE_REAR:
        return "rear";
    case ESCAPE_LEFT:
        return "left";
    case ESCAPE_RIGHT:
        return "right";
    default:
        return "none";
    }
}

}  // namespace cym_planner

#endif  // CYM_PLANNER_ESCAPE_RECOVERY_H_
