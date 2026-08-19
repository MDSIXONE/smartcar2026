#include "cym_planner.h"
#include "cym_planner/global_cost_semantics.h"
#include "cym_planner/local_elastic_path.h"

#include <cv_bridge/cv_bridge.h>
#include <boost/thread/locks.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cctype>
#include <clocale>
#include <cmath>
#include <limits>
#include <vector>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace
{

const double kTargetDistance = 0.20;
const double kPi = 3.14159265358979323846;

double clampValue(double value, double minimum, double maximum)
{
    return cym_planner::finiteClamp(value, minimum, maximum);
}

bool poseIsFinite(const geometry_msgs::PoseStamped& pose)
{
    return std::isfinite(pose.pose.position.x) &&
        std::isfinite(pose.pose.position.y) &&
        std::isfinite(pose.pose.position.z) &&
        std::isfinite(pose.pose.orientation.x) &&
        std::isfinite(pose.pose.orientation.y) &&
        std::isfinite(pose.pose.orientation.z) &&
        std::isfinite(pose.pose.orientation.w);
}

bool commandIsFinite(const geometry_msgs::Twist& command)
{
    return std::isfinite(command.linear.x) &&
        std::isfinite(command.linear.y) &&
        std::isfinite(command.angular.z);
}

std::string normalizedFrameId(const std::string& frame_id)
{
    const std::string::size_type first =
        frame_id.find_first_not_of('/');
    return first == std::string::npos ?
        std::string() : frame_id.substr(first);
}

class ControlCycleWatchdog
{
public:
    ControlCycleWatchdog()
        : started_(ros::WallTime::now())
    {
    }

    ~ControlCycleWatchdog()
    {
        const double elapsed =
            (ros::WallTime::now() - started_).toSec();
        if(elapsed > 0.05)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "cym_planner: control cycle exceeded 50 ms: %.1f ms",
                elapsed * 1000.0);
        }
    }

private:
    ros::WallTime started_;
};

double normalizeAngle(double angle)
{
    while(angle > kPi)
        angle -= 2.0 * kPi;
    while(angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

template<typename T>
void readPlannerParam(const ros::NodeHandle& primary,
                      const ros::NodeHandle& canonical,
                      const ros::NodeHandle& legacy,
                      const std::string& key,
                      T& value,
                      const T& default_value)
{
    if(primary.getParam(key, value) ||
       canonical.getParam(key, value) ||
       legacy.getParam(key, value))
    {
        return;
    }
    value = default_value;
}

cym_planner::PlannerTuning pointDefaults()
{
    cym_planner::PlannerTuning tuning;
    tuning.linear_x_gain = 1.5;
    tuning.linear_x_kd = 0.5;
    tuning.angular_gain = 2.5;
    tuning.angular_kd = 0.4;
    tuning.max_vel_x = 0.5;
    tuning.max_vel_theta = 1.0;
    tuning.final_yaw_gain = 2.0;
    tuning.final_yaw_max_vel = 1.0;
    tuning.final_yaw_tolerance = 0.10;
    tuning.final_linear_x_gain = 1.0;
    tuning.goal_position_tolerance = 0.07;
    tuning.obstacle_lookahead_distance = 0.25;
    tuning.obstacle_cost_threshold = 253;
    tuning.carry_speed_scale = 1.0;
    tuning.heading_slowdown_min_scale = 1.0;
    tuning.command_sweep_time = 0.0;
    tuning.command_sweep_step = 0.025;
    return tuning;
}

cym_planner::PlannerTuning bodyProjectionDefaults()
{
    cym_planner::PlannerTuning tuning;
    tuning.linear_x_gain = 0.9;
    tuning.linear_x_kd = 0.2;
    tuning.angular_gain = 2.0;
    tuning.angular_kd = 0.2;
    tuning.max_vel_x = 0.22;
    tuning.max_vel_theta = 0.55;
    tuning.final_yaw_gain = 1.5;
    tuning.final_yaw_max_vel = 0.35;
    tuning.final_yaw_tolerance = 0.10;
    tuning.final_linear_x_gain = 0.6;
    tuning.obstacle_lookahead_distance = 0.30;
    tuning.obstacle_cost_threshold = 253;
    tuning.carry_speed_scale = 1.0;
    tuning.heading_slowdown_min_scale = 0.15;
    tuning.command_sweep_time = 0.40;
    tuning.command_sweep_step = 0.025;
    return tuning;
}

cym_planner::PlannerTuning sprintDefaults()
{
    cym_planner::PlannerTuning tuning;
    tuning.linear_x_gain = 12.5;
    tuning.linear_x_kd = 0.5;
    tuning.angular_gain = 10.0;
    tuning.angular_kd = 0.4;
    tuning.max_vel_x = 2.5;
    tuning.max_vel_theta = 0.80;
    tuning.final_yaw_gain = 2.0;
    tuning.final_yaw_max_vel = 1.0;
    tuning.final_yaw_tolerance = 0.10;
    tuning.final_linear_x_gain = 0.6;
    tuning.obstacle_lookahead_distance = 0.25;
    tuning.obstacle_cost_threshold = 253;
    tuning.carry_speed_scale = 1.0;
    tuning.heading_slowdown_min_scale = 0.0;
    tuning.command_sweep_time = 0.0;
    tuning.command_sweep_step = 0.025;
    tuning.approach_decel_distance = 1.0;
    tuning.approach_min_vel_x = 0.12;
    tuning.lateral_gain = 12.5;
    tuning.max_vel_y = 2.5;
    return tuning;
}

cym_planner::PlannerTuning destinationDefaults()
{
    cym_planner::PlannerTuning tuning = pointDefaults();
    tuning.final_linear_x_gain = 1.0;
    tuning.goal_position_tolerance = 0.04;
    return tuning;
}

void readTuning(
    const ros::NodeHandle& primary,
    const ros::NodeHandle& canonical,
    const ros::NodeHandle& legacy,
    const std::string& prefix,
    const cym_planner::PlannerTuning& defaults,
    cym_planner::PlannerTuning& tuning)
{
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/linear_x_gain",
                     tuning.linear_x_gain, defaults.linear_x_gain);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/linear_x_kd",
                     tuning.linear_x_kd, defaults.linear_x_kd);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/angular_gain",
                     tuning.angular_gain, defaults.angular_gain);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/angular_kd",
                     tuning.angular_kd, defaults.angular_kd);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/max_vel_x",
                     tuning.max_vel_x, defaults.max_vel_x);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/max_vel_theta",
                     tuning.max_vel_theta, defaults.max_vel_theta);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/final_yaw_gain",
                     tuning.final_yaw_gain, defaults.final_yaw_gain);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/final_yaw_max_vel",
                     tuning.final_yaw_max_vel,
                     defaults.final_yaw_max_vel);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/final_yaw_tolerance",
                     tuning.final_yaw_tolerance,
                     defaults.final_yaw_tolerance);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/final_linear_x_gain",
                     tuning.final_linear_x_gain,
                     defaults.final_linear_x_gain);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/goal_position_tolerance",
                     tuning.goal_position_tolerance,
                     defaults.goal_position_tolerance);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/obstacle_lookahead_distance",
                     tuning.obstacle_lookahead_distance,
                     defaults.obstacle_lookahead_distance);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/obstacle_cost_threshold",
                     tuning.obstacle_cost_threshold,
                     defaults.obstacle_cost_threshold);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/carry_speed_scale",
                     tuning.carry_speed_scale,
                     defaults.carry_speed_scale);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/heading_slowdown_min_scale",
                     tuning.heading_slowdown_min_scale,
                     defaults.heading_slowdown_min_scale);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/command_sweep_time",
                     tuning.command_sweep_time,
                     defaults.command_sweep_time);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/command_sweep_step",
                     tuning.command_sweep_step,
                     defaults.command_sweep_step);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/approach_decel_distance",
                     tuning.approach_decel_distance,
                     defaults.approach_decel_distance);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/approach_min_vel_x",
                     tuning.approach_min_vel_x,
                     defaults.approach_min_vel_x);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/lateral_gain",
                     tuning.lateral_gain,
                     defaults.lateral_gain);
    readPlannerParam(primary, canonical, legacy,
                     prefix + "/max_vel_y",
                     tuning.max_vel_y,
                     defaults.max_vel_y);
}

void sanitizeTuning(cym_planner::PlannerTuning& tuning)
{
    tuning.linear_x_gain = std::max(0.0, tuning.linear_x_gain);
    tuning.linear_x_kd = std::max(0.0, tuning.linear_x_kd);
    tuning.angular_gain = std::max(0.0, tuning.angular_gain);
    tuning.angular_kd = std::max(0.0, tuning.angular_kd);
    tuning.max_vel_x = std::max(0.0, tuning.max_vel_x);
    tuning.max_vel_theta = std::max(0.0, tuning.max_vel_theta);
    tuning.final_yaw_gain = std::max(0.0, tuning.final_yaw_gain);
    tuning.final_yaw_max_vel =
        std::max(0.0, tuning.final_yaw_max_vel);
    tuning.final_yaw_tolerance =
        clampValue(tuning.final_yaw_tolerance, 0.01, kPi);
    tuning.final_linear_x_gain =
        std::max(0.0, tuning.final_linear_x_gain);
    tuning.goal_position_tolerance = clampValue(
        tuning.goal_position_tolerance, 0.01, 1.0);
    tuning.obstacle_lookahead_distance =
        std::max(0.0, tuning.obstacle_lookahead_distance);
    tuning.obstacle_cost_threshold = static_cast<int>(
        clampValue(
            static_cast<double>(tuning.obstacle_cost_threshold),
            0.0, 255.0));
    tuning.carry_speed_scale =
        clampValue(tuning.carry_speed_scale, 0.05, 1.0);
    tuning.heading_slowdown_min_scale =
        clampValue(tuning.heading_slowdown_min_scale, 0.0, 1.0);
    tuning.command_sweep_time =
        clampValue(tuning.command_sweep_time, 0.0, 1.0);
    if(!std::isfinite(tuning.command_sweep_step) ||
       tuning.command_sweep_step <= 0.0)
    {
        ROS_ERROR(
            "cym_planner: command_sweep_step must be finite and positive; "
            "forcing safety default 0.025 s");
        tuning.command_sweep_step = 0.025;
    }
    tuning.command_sweep_step =
        clampValue(tuning.command_sweep_step, 0.005, 0.05);
    tuning.approach_decel_distance =
        clampValue(tuning.approach_decel_distance, 0.0, 2.0);
    tuning.approach_min_vel_x =
        clampValue(tuning.approach_min_vel_x, 0.0, 1.0);
    tuning.lateral_gain = std::max(0.0, tuning.lateral_gain);
    tuning.max_vel_y = clampValue(tuning.max_vel_y, 0.0, 3.0);
}

}  // namespace

namespace cym_planner
{

CymPlanner::CymPlanner()
    : initialized_(false),
      tf_listener_(NULL),
      costmap_ros_(NULL),
      debug_images_enabled_(false),
      escape_enabled_(false),
      elastic_enabled_(true),
      elastic_lookahead_distance_(0.25),
      elastic_lateral_step_(0.02),
      elastic_max_lateral_offset_(0.10),
      elastic_validation_step_(0.015),
      elastic_validation_yaw_step_(0.05),
      elastic_max_vel_x_(0.07),
      elastic_max_vel_theta_(0.30),
      elastic_search_timeout_(0.40),
      elastic_activation_cost_(220),
      target_index_(0),
      pose_adjusting_(false),
      goal_reached_(false),
      carry_mode_(false),
      body_projection_enabled_(false),
      sprint_enabled_(false),
      transverse_enabled_(false),
      destination_enabled_(false),
      elastic_active_(false),
      elastic_end_plan_index_(-1),
      elastic_last_side_(0),
      elastic_blocked_since_(ros::Time(0)),
      escape_blocked_since_(ros::Time(0)),
      escape_wait_until_(ros::Time(0)),
      escape_motion_started_(ros::Time(0)),
      escape_active_(false),
      escape_attempts_(0),
      escape_total_distance_(0.0),
      escape_start_world_x_(0.0),
      escape_start_world_y_(0.0),
      escape_start_world_yaw_(0.0),
      escape_direction_base_x_(0.0),
      escape_direction_base_y_(0.0),
      escape_direction_world_x_(0.0),
      escape_direction_world_y_(0.0),
      previous_linear_error_(0.0),
      previous_linear_control_time_(ros::Time(0)),
      linear_derivative_initialized_(false),
      previous_heading_error_(0.0),
      previous_heading_control_time_(ros::Time(0)),
      angular_derivative_initialized_(false)
{
    setlocale(LC_ALL, "");
}

CymPlanner::~CymPlanner()
{
    delete tf_listener_;
}

void CymPlanner::initialize(std::string name, tf2_ros::Buffer* /* tf */,
                            costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_)
        return;

    tf_listener_ = new tf::TransformListener();
    costmap_ros_ = costmap_ros;

    ros::NodeHandle planner_nh("~/" + name);
    ros::NodeHandle canonical_nh("~/cym_planner/CymPlanner");
    ros::NodeHandle legacy_nh("~/CymPlanner");

    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "base_link_frame", base_link_frame_, std::string("base_link"));
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "odom_frame", odom_frame_, std::string("odom"));
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "debug_images_enabled", debug_images_enabled_, false);
    readTuning(
        planner_nh, canonical_nh, legacy_nh,
        "mode1_point", pointDefaults(), point_tuning_);
    readTuning(
        planner_nh, canonical_nh, legacy_nh,
        "mode2_body_projection", bodyProjectionDefaults(),
        body_projection_tuning_);
    readTuning(
        planner_nh, canonical_nh, legacy_nh,
        "mode3_sprint", sprintDefaults(), sprint_tuning_);
    readTuning(
        planner_nh, canonical_nh, legacy_nh,
        "mode4_destination", destinationDefaults(), destination_tuning_);
    sanitizeTuning(point_tuning_);
    sanitizeTuning(body_projection_tuning_);
    sanitizeTuning(sprint_tuning_);
    sanitizeTuning(destination_tuning_);
    if(body_projection_tuning_.command_sweep_time <= 0.0)
    {
        ROS_ERROR(
            "cym_planner: mode2 command_sweep_time must be positive; "
            "forcing safety default 0.40 s");
        body_projection_tuning_.command_sweep_time = 0.40;
    }
    bool requested_escape_enabled = false;
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_enabled", requested_escape_enabled, false);
    escape_enabled_ = false;
    if(requested_escape_enabled)
    {
        ROS_ERROR(
            "cym_planner: escape_enabled=true was requested but is ignored; "
            "bounded escape remains hard-disabled pending safety fixes");
    }
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_blocked_timeout", escape_blocked_timeout_, 0.4);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_replan_wait", escape_replan_wait_, 0.4);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_step_distance", escape_step_distance_, 0.02);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_projection_step", escape_projection_step_, 0.005);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_speed", escape_speed_, 0.04);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_max_total_distance",
                     escape_max_total_distance_, 0.08);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_heading_tolerance",
                     escape_heading_tolerance_, 0.05);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "escape_max_attempts", escape_max_attempts_, 4);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_enabled", elastic_enabled_, true);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_lookahead_distance",
                     elastic_lookahead_distance_, 0.25);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_lateral_step", elastic_lateral_step_, 0.02);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_max_lateral_offset",
                     elastic_max_lateral_offset_, 0.10);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_validation_step", elastic_validation_step_,
                     0.015);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_validation_yaw_step",
                     elastic_validation_yaw_step_, 0.05);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_max_vel_x", elastic_max_vel_x_, 0.07);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_max_vel_theta", elastic_max_vel_theta_, 0.30);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_search_timeout", elastic_search_timeout_, 0.40);
    readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                     "elastic_activation_cost", elastic_activation_cost_, 220);

    escape_blocked_timeout_ =
        clampValue(escape_blocked_timeout_, 0.10, 2.0);
    escape_replan_wait_ = clampValue(escape_replan_wait_, 0.10, 2.0);
    escape_step_distance_ = clampValue(escape_step_distance_, 0.005, 0.03);
    escape_projection_step_ =
        clampValue(escape_projection_step_, 0.002, 0.01);
    escape_speed_ = clampValue(escape_speed_, 0.01, 0.08);
    escape_max_total_distance_ =
        clampValue(escape_max_total_distance_, escape_step_distance_, 0.10);
    escape_heading_tolerance_ =
        clampValue(escape_heading_tolerance_, 0.02, 0.10);
    escape_max_attempts_ = std::max(1, std::min(10, escape_max_attempts_));
    elastic_lookahead_distance_ =
        clampValue(elastic_lookahead_distance_, 0.20, 0.30);
    elastic_lateral_step_ =
        clampValue(elastic_lateral_step_, 0.01, 0.03);
    elastic_max_lateral_offset_ =
        clampValue(elastic_max_lateral_offset_, elastic_lateral_step_, 0.10);
    elastic_validation_step_ =
        clampValue(elastic_validation_step_, 0.005, 0.015);
    elastic_validation_yaw_step_ =
        clampValue(elastic_validation_yaw_step_, 0.02, 0.10);
    elastic_max_vel_x_ = clampValue(elastic_max_vel_x_, 0.02, 0.07);
    elastic_max_vel_theta_ = clampValue(elastic_max_vel_theta_, 0.05, 0.30);
    elastic_search_timeout_ =
        clampValue(elastic_search_timeout_, 0.10, 1.00);
    elastic_activation_cost_ = std::max(
        1, std::min(253, elastic_activation_cost_));

    ros::NodeHandle public_nh;
    carry_mode_sub_ = public_nh.subscribe(
        "/ucar/carry_mode", 1, &CymPlanner::carryModeCallback, this);
    navigation_mode_sub_ = public_nh.subscribe(
        "/ucar/navigation_mode", 1, &CymPlanner::navigationModeCallback, this);
    if(debug_images_enabled_)
    {
        debug_map_pub_ =
            canonical_nh.advertise<sensor_msgs::Image>("debug_map", 1);
        debug_plan_pub_ =
            canonical_nh.advertise<sensor_msgs::Image>("debug_plan", 1);
    }

    initialized_ = true;
    ROS_WARN(
        "cym_planner initialized | mode1 point max %.2f m/s %.2f rad/s | "
        "mode2 body max %.2f m/s %.2f rad/s turn_scale %.2f | "
        "debug images %s | escape %s | elastic path %s (%.2f m band)",
        point_tuning_.max_vel_x, point_tuning_.max_vel_theta,
        body_projection_tuning_.max_vel_x,
        body_projection_tuning_.max_vel_theta,
        body_projection_tuning_.heading_slowdown_min_scale,
        debug_images_enabled_ ? "enabled" : "disabled",
        escape_enabled_ ? "enabled" : "disabled",
        elastic_enabled_ ? "enabled" : "disabled",
        elastic_max_lateral_offset_);
    ROS_WARN("cym_planner mode3 sprint max %.2f m/s %.2f rad/s",
             sprint_tuning_.max_vel_x, sprint_tuning_.max_vel_theta);
    ROS_WARN("cym_planner mode3 sprint lateral max %.2f m/s",
             sprint_tuning_.max_vel_y);
}

void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
{
    if(carry_mode_ == message->data)
        return;
    carry_mode_ = message->data;
    const PlannerTuning& tuning = activeTuning();
    ROS_WARN("cym_planner carry mode %s; speed scale %.2f",
             carry_mode_ ? "enabled" : "disabled",
             carry_mode_ ? tuning.carry_speed_scale : 1.0);
}

void CymPlanner::navigationModeCallback(const std_msgs::String::ConstPtr& message)
{
    std::string mode = message->data;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });

    bool requested_body_mode = body_projection_enabled_;
    bool requested_sprint_mode = sprint_enabled_;
    bool requested_transverse_mode = transverse_enabled_;
    bool requested_destination_mode = destination_enabled_;
    if(mode == "destination" || mode == "handoff" || mode == "final")
    {
        requested_body_mode = false;
        requested_sprint_mode = false;
        requested_transverse_mode = false;
        requested_destination_mode = true;
    }
    else if(mode == "body" || mode == "body_projection" || mode == "footprint" ||
       mode == "laser_avoidance")
    {
        requested_body_mode = true;
        requested_sprint_mode = false;
        requested_transverse_mode = false;
        requested_destination_mode = false;
    }
    else if(mode == "point" || mode == "path_point" || mode == "direct_line" ||
            mode == "main" || mode == "main_legacy")
    {
        requested_body_mode = false;
        requested_sprint_mode = false;
        requested_transverse_mode = false;
        requested_destination_mode = false;
    }
    else if(mode == "sprint" || mode == "fast")
    {
        requested_body_mode = false;
        requested_sprint_mode = true;
        requested_transverse_mode = false;
        requested_destination_mode = false;
    }
    else if(mode == "transverse" || mode == "lateral" || mode == "strafe")
    {
        requested_body_mode = false;
        requested_sprint_mode = false;
        requested_transverse_mode = true;
        requested_destination_mode = false;
    }
    else
    {
        ROS_WARN("cym_planner: unsupported navigation mode '%s'; "
                 "use point, destination, body_projection, sprint or "
                 "transverse",
                 message->data.c_str());
        return;
    }

    if(requested_body_mode == body_projection_enabled_ &&
       requested_sprint_mode == sprint_enabled_ &&
       requested_transverse_mode == transverse_enabled_ &&
       requested_destination_mode == destination_enabled_)
        return;
    body_projection_enabled_ = requested_body_mode;
    sprint_enabled_ = requested_sprint_mode;
    transverse_enabled_ = requested_transverse_mode;
    destination_enabled_ = requested_destination_mode;
    resetControllerState();
    resetEscapeRecovery();
    clearElasticPlan();
    resetElasticSearch();
    const PlannerTuning& tuning = activeTuning();
    ROS_WARN(
        "cym_planner switched to %s | linear P/D %.2f/%.2f "
        "angular P/D %.2f/%.2f max %.2f m/s %.2f rad/s",
        destination_enabled_ ? "mode4_destination" :
            (sprint_enabled_ ? "mode3_sprint" :
                (transverse_enabled_ ? "mode3_sprint (transverse)" :
                    (body_projection_enabled_ ?
                        "mode2_body_projection" : "mode1_point"))),
        tuning.linear_x_gain, tuning.linear_x_kd,
        tuning.angular_gain, tuning.angular_kd,
        tuning.max_vel_x, tuning.max_vel_theta);
}

void CymPlanner::resetControllerState()
{
    previous_linear_error_ = 0.0;
    previous_linear_control_time_ = ros::Time(0);
    linear_derivative_initialized_ = false;
    previous_heading_error_ = 0.0;
    previous_heading_control_time_ = ros::Time(0);
    angular_derivative_initialized_ = false;
}

const PlannerTuning& CymPlanner::activeTuning() const
{
    if(destination_enabled_)
        return destination_tuning_;
    if(sprint_enabled_ || transverse_enabled_)
        return sprint_tuning_;
    return selectPlannerTuning(
        body_projection_enabled_,
        point_tuning_,
        body_projection_tuning_);
}

void CymPlanner::resetEscapeRecovery()
{
    escape_blocked_since_ = ros::Time(0);
    escape_wait_until_ = ros::Time(0);
    escape_motion_started_ = ros::Time(0);
    escape_active_ = false;
    escape_attempts_ = 0;
    escape_total_distance_ = 0.0;
    escape_start_world_x_ = 0.0;
    escape_start_world_y_ = 0.0;
    escape_start_world_yaw_ = 0.0;
    escape_direction_base_x_ = 0.0;
    escape_direction_base_y_ = 0.0;
    escape_direction_world_x_ = 0.0;
    escape_direction_world_y_ = 0.0;
}

bool CymPlanner::currentCostmapPose(
    geometry_msgs::PoseStamped& pose) const
{
    return costmap_ros_->getRobotPose(pose) && poseIsFinite(pose);
}

bool CymPlanner::commandSweepIsSafe(
    const geometry_msgs::Twist& command,
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image)
{
    if(!commandIsFinite(command))
        return false;
    const PlannerTuning& tuning = activeTuning();
    if(tuning.command_sweep_time <= 0.0)
        return true;

    geometry_msgs::PoseStamped pose;
    if(!currentCostmapPose(pose))
        return false;
    const int steps = std::max(
        1, static_cast<int>(std::ceil(
            tuning.command_sweep_time / tuning.command_sweep_step)));
    const double period =
        tuning.command_sweep_time / static_cast<double>(steps);
    double yaw = tf::getYaw(pose.pose.orientation);
    for(int step = 0; step <= steps; ++step)
    {
        if(step > 0)
        {
            const double cosine = std::cos(yaw);
            const double sine = std::sin(yaw);
            pose.pose.position.x +=
                (cosine * command.linear.x -
                 sine * command.linear.y) * period;
            pose.pose.position.y +=
                (sine * command.linear.x +
                 cosine * command.linear.y) * period;
            yaw = normalizeAngle(yaw + command.angular.z * period);
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        const FootprintBlockage blockage =
            inspectFootprint(pose, local_costmap, map_image, false);
        if(blockage.blocked)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "cym_planner: candidate command footprint sweep blocked "
                "at %.3f s",
                static_cast<double>(step) * period);
            return false;
        }
    }
    return true;
}

bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    const bool preserve_elastic = elasticPlanMatchesNewGlobal(plan);
    const bool preserve_search = elasticSearchTimerSurvivesEquivalentPlan(
        !elastic_blocked_since_.isZero(),
        elasticSearchPlanMatchesNewGlobal(plan));
    global_plan_ = plan;
    if(!preserve_elastic)
    {
        clearElasticPlan();
        if(!preserve_search)
            resetElasticSearch();
        target_index_ = 0;
        pose_adjusting_ = false;
        goal_reached_ = false;
        resetControllerState();
    }
    return !global_plan_.empty();
}

const std::vector<geometry_msgs::PoseStamped>& CymPlanner::trackingPlan() const
{
    return elastic_active_ ? elastic_plan_ : global_plan_;
}

void CymPlanner::clearElasticPlan()
{
    elastic_active_ = false;
    elastic_end_plan_index_ = -1;
    elastic_plan_.clear();
}

void CymPlanner::resetElasticSearch()
{
    elastic_blocked_since_ = ros::Time(0);
    elastic_search_reference_plan_.clear();
}

bool CymPlanner::plansHaveSameGeometry(
    const std::vector<geometry_msgs::PoseStamped>& first,
    const std::vector<geometry_msgs::PoseStamped>& plan) const
{
    if(first.empty() || plan.empty() ||
       normalizedFrameId(first.front().header.frame_id) !=
       normalizedFrameId(plan.front().header.frame_id))
    {
        return false;
    }
    for(const geometry_msgs::PoseStamped& pose : first)
    {
        if(!poseIsFinite(pose) ||
           normalizedFrameId(pose.header.frame_id) !=
           normalizedFrameId(first.front().header.frame_id))
            return false;
    }
    for(const geometry_msgs::PoseStamped& pose : plan)
    {
        if(!poseIsFinite(pose) ||
           normalizedFrameId(pose.header.frame_id) !=
           normalizedFrameId(plan.front().header.frame_id))
            return false;
    }
    const auto planLength = [](const std::vector<geometry_msgs::PoseStamped>& path)
    {
        double length = 0.0;
        for(int index = 1; index < static_cast<int>(path.size()); ++index)
        {
            length += std::hypot(
                path[index].pose.position.x - path[index - 1].pose.position.x,
                path[index].pose.position.y - path[index - 1].pose.position.y);
        }
        return length;
    };
    const auto samplePlan = [](
        const std::vector<geometry_msgs::PoseStamped>& path,
        double requested_distance,
        double& world_x, double& world_y, double& yaw)
    {
        double travelled = 0.0;
        for(int index = 1; index < static_cast<int>(path.size()); ++index)
        {
            const double delta_x = path[index].pose.position.x -
                path[index - 1].pose.position.x;
            const double delta_y = path[index].pose.position.y -
                path[index - 1].pose.position.y;
            const double segment = std::hypot(delta_x, delta_y);
            if(segment < 1e-6)
                continue;
            if(travelled + segment >= requested_distance ||
               index == static_cast<int>(path.size()) - 1)
            {
                const double ratio = std::max(0.0, std::min(
                    1.0, (requested_distance - travelled) / segment));
                world_x = path[index - 1].pose.position.x + delta_x * ratio;
                world_y = path[index - 1].pose.position.y + delta_y * ratio;
                yaw = std::atan2(delta_y, delta_x);
                return true;
            }
            travelled += segment;
        }
        return false;
    };
    const double first_length = planLength(first);
    const double second_length = planLength(plan);
    if(!std::isfinite(first_length) || !std::isfinite(second_length) ||
       first_length < 1e-6 || second_length < 1e-6)
    {
        return std::hypot(first.back().pose.position.x - plan.back().pose.position.x,
                          first.back().pose.position.y - plan.back().pose.position.y) <= 0.03;
    }
    for(int sample = 0; sample <= 6; ++sample)
    {
        const double ratio = static_cast<double>(sample) / 6.0;
        double first_x = 0.0;
        double first_y = 0.0;
        double first_yaw = 0.0;
        double second_x = 0.0;
        double second_y = 0.0;
        double second_yaw = 0.0;
        if(!samplePlan(first, first_length * ratio, first_x, first_y, first_yaw) ||
           !samplePlan(plan, second_length * ratio,
                       second_x, second_y, second_yaw) ||
           std::hypot(first_x - second_x, first_y - second_y) > 0.04 ||
           std::abs(normalizeAngle(first_yaw - second_yaw)) > 0.20)
        {
            return false;
        }
    }
    return true;
}

bool CymPlanner::elasticPlanMatchesNewGlobal(
    const std::vector<geometry_msgs::PoseStamped>& plan) const
{
    return elastic_active_ && plansHaveSameGeometry(global_plan_, plan);
}

bool CymPlanner::elasticSearchPlanMatchesNewGlobal(
    const std::vector<geometry_msgs::PoseStamped>& plan) const
{
    return !elastic_search_reference_plan_.empty() &&
        plansHaveSameGeometry(elastic_search_reference_plan_, plan);
}

bool CymPlanner::transformPlanPose(
    const geometry_msgs::PoseStamped& source,
    const std::string& target_frame,
    geometry_msgs::PoseStamped& result) const
{
    geometry_msgs::PoseStamped unstamped = source;
    unstamped.header.stamp = ros::Time(0);
    try
    {
        tf_listener_->transformPose(target_frame, unstamped, result);
        return true;
    }
    catch(const tf::TransformException& error)
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform plan pose to %s: %s",
                          target_frame.c_str(), error.what());
        return false;
    }
}

bool CymPlanner::selectTargetPose(geometry_msgs::PoseStamped& target_pose)
{
    const std::vector<geometry_msgs::PoseStamped>& plan = trackingPlan();
    bool have_target = false;
    for(int index = target_index_;
        index < static_cast<int>(plan.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_base;
        if(!transformPlanPose(plan[index], base_link_frame_, pose_base))
            return false;

        target_pose = pose_base;
        have_target = true;
        const double distance = std::hypot(
            pose_base.pose.position.x, pose_base.pose.position.y);
        if(distance > kTargetDistance ||
           index == static_cast<int>(plan.size()) - 1)
        {
            target_index_ = index;
            break;
        }
    }
    return have_target;
}

CymPlanner::FootprintBlockage CymPlanner::inspectFootprint(
    const geometry_msgs::PoseStamped& pose_costmap,
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image,
    bool report_contact)
{
    FootprintBlockage result;
    if(!poseIsFinite(pose_costmap))
    {
        result.blocked = true;
        return result;
    }
    if(normalizedFrameId(pose_costmap.header.frame_id) !=
       normalizedFrameId(costmap_ros_->getGlobalFrameID()))
    {
        ROS_ERROR_THROTTLE(
            1.0,
            "cym_planner: footprint frame '%s' does not match local "
            "costmap frame '%s'",
            pose_costmap.header.frame_id.c_str(),
            costmap_ros_->getGlobalFrameID().c_str());
        result.blocked = true;
        return result;
    }
    if(local_costmap.getResolution() <= 0.0 ||
       local_costmap.getSizeInCellsX() == 0 ||
       local_costmap.getSizeInCellsY() == 0)
    {
        ROS_ERROR_THROTTLE(
            1.0, "cym_planner: invalid local costmap snapshot");
        result.blocked = true;
        return result;
    }

    const std::vector<geometry_msgs::Point>& footprint =
        costmap_ros_->getRobotFootprint();
    if(footprint.size() < 3)
    {
        ROS_WARN_THROTTLE(
            1.0, "cym_planner: invalid footprint; body projection stopped");
        result.blocked = true;
        return result;
    }

    const double yaw = tf::getYaw(pose_costmap.pose.orientation);
    if(!std::isfinite(yaw))
    {
        result.blocked = true;
        return result;
    }
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    std::vector<cv::Point> polygon;
    polygon.reserve(footprint.size());
    for(const geometry_msgs::Point& point : footprint)
    {
        if(!std::isfinite(point.x) || !std::isfinite(point.y))
        {
            result.blocked = true;
            return result;
        }
        const double world_x = pose_costmap.pose.position.x +
            cosine * point.x - sine * point.y;
        const double world_y = pose_costmap.pose.position.y +
            sine * point.x + cosine * point.y;
        unsigned int map_x = 0;
        unsigned int map_y = 0;
        if(!local_costmap.worldToMap(world_x, world_y, map_x, map_y))
        {
            ROS_WARN_THROTTLE(
                1.0, "cym_planner: projected footprint leaves local costmap");
            result.blocked = true;
            return result;
        }
        polygon.push_back(cv::Point(
            static_cast<int>(map_x), static_cast<int>(map_y)));
    }

    const cv::Rect bounds = cv::boundingRect(polygon);
    const int maximum_x = std::min(
        bounds.x + bounds.width,
        static_cast<int>(local_costmap.getSizeInCellsX()));
    const int maximum_y = std::min(
        bounds.y + bounds.height,
        static_cast<int>(local_costmap.getSizeInCellsY()));
    for(int y = std::max(0, bounds.y); y < maximum_y; ++y)
    {
        for(int x = std::max(0, bounds.x); x < maximum_x; ++x)
        {
            if(cv::pointPolygonTest(
                   polygon, cv::Point2f(x + 0.5f, y + 0.5f), false) < 0.0)
            {
                continue;
            }
            const unsigned char cost = local_costmap.getCost(
                static_cast<unsigned int>(x),
                static_cast<unsigned int>(y));
            result.maximum_cost = std::max(
                result.maximum_cost, static_cast<unsigned int>(cost));
            result.total_cost += static_cast<std::uint64_t>(cost);
            ++result.sampled_cells;
            if(localCellBlocksProjectedFootprint(cost))
            {
                double blocked_world_x = 0.0;
                double blocked_world_y = 0.0;
                local_costmap.mapToWorld(
                    static_cast<unsigned int>(x),
                    static_cast<unsigned int>(y),
                    blocked_world_x, blocked_world_y);
                result.blocked = true;
                result.recoverable = cost == 254;
                result.contact_count = result.recoverable ? 1u : 0u;
                result.contact_world_x = blocked_world_x;
                result.contact_world_y = blocked_world_y;
                if(report_contact)
                {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "cym_planner: projected footprint contacts local %s cell "
                        "at (%.3f, %.3f), cost=%u",
                        cost == 255 ? "unknown" : "lethal",
                        blocked_world_x, blocked_world_y,
                        static_cast<unsigned int>(cost));
                }
                if(!map_image.empty())
                {
                    cv::circle(
                        map_image,
                        cv::Point(x, y),
                        2, cv::Scalar(0, 0, 255), -1);
                }
                if(report_contact)
                {
                    ROS_DEBUG_THROTTLE(
                        1.0,
                        "cym_planner body projection pose=(%.3f, %.3f, %.3f)",
                        pose_costmap.pose.position.x,
                        pose_costmap.pose.position.y,
                        yaw);
                }
                return result;
            }
        }
    }
    return result;
}

bool CymPlanner::elasticCandidateClearance(
    const std::vector<geometry_msgs::PoseStamped>& candidate,
    int start_index,
    int end_index,
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image,
    ElasticClearanceScore& score)
{
    const double absolute_offset = score.absolute_offset;
    score = ElasticClearanceScore();
    score.absolute_offset = absolute_offset;
    if(start_index < 0 || end_index <= start_index ||
       end_index >= static_cast<int>(candidate.size()))
    {
        return false;
    }
    geometry_msgs::PoseStamped previous = candidate[start_index];
    FootprintBlockage footprint =
        inspectFootprint(previous, local_costmap, map_image, false);
    if(footprint.blocked)
        return false;
    // The immutable current pose must be collision-free, but it must not
    // dominate the ranking of all candidates.  A band is useful precisely
    // when it moves a still-safe footprint away from high inflation ahead.
    for(int index = start_index + 1; index <= end_index; ++index)
    {
        const geometry_msgs::PoseStamped& next = candidate[index];
        const double distance = std::hypot(
            next.pose.position.x - previous.pose.position.x,
            next.pose.position.y - previous.pose.position.y);
        if(!std::isfinite(distance))
            return false;
        const double previous_yaw = tf::getYaw(previous.pose.orientation);
        const double next_yaw = tf::getYaw(next.pose.orientation);
        if(!std::isfinite(previous_yaw) || !std::isfinite(next_yaw))
            return false;
        const double yaw_delta = normalizeAngle(next_yaw - previous_yaw);
        const int samples = elasticInterpolationSamples(
            distance, elastic_validation_step_, yaw_delta,
            elastic_validation_yaw_step_);
        if(samples <= 0)
            return false;
        for(int sample = 1; sample <= samples; ++sample)
        {
            const double ratio = static_cast<double>(sample) /
                static_cast<double>(samples);
            geometry_msgs::PoseStamped probe = next;
            probe.pose.position.x = previous.pose.position.x +
                (next.pose.position.x - previous.pose.position.x) * ratio;
            probe.pose.position.y = previous.pose.position.y +
                (next.pose.position.y - previous.pose.position.y) * ratio;
            probe.pose.orientation = tf::createQuaternionMsgFromYaw(
                normalizeAngle(previous_yaw + yaw_delta * ratio));
            footprint = inspectFootprint(probe, local_costmap, map_image, false);
            if(footprint.blocked)
                return false;
            score.maximum_cost = std::max(
                score.maximum_cost, footprint.maximum_cost);
            score.total_cost += footprint.total_cost;
            score.sampled_cells += footprint.sampled_cells;
        }
        previous = next;
    }
    score.valid = score.sampled_cells > 0;
    return score.valid;
}

bool CymPlanner::tryActivateElasticPlan(
    const std::vector<geometry_msgs::PoseStamped>& costmap_plan,
    int start_index,
    const geometry_msgs::PoseStamped& current_pose,
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image,
    unsigned int maximum_accepted_cost)
{
    if(!body_projection_enabled_ || !elastic_enabled_ || elastic_active_ ||
       costmap_plan.size() != global_plan_.size() ||
       start_index < 0 || start_index + 2 >= static_cast<int>(costmap_plan.size()))
    {
        return false;
    }
    int end_index = start_index;
    double travelled = 0.0;
    for(int index = start_index + 1;
        index < static_cast<int>(costmap_plan.size()); ++index)
    {
        travelled += std::hypot(
            costmap_plan[index].pose.position.x -
                costmap_plan[index - 1].pose.position.x,
            costmap_plan[index].pose.position.y -
                costmap_plan[index - 1].pose.position.y);
        end_index = index;
        if(travelled >= elastic_lookahead_distance_)
            break;
    }
    if(end_index <= start_index + 1 ||
       travelled < std::min(0.30, elastic_lookahead_distance_ * 0.75))
        return false;

    const int preferred_side = elastic_last_side_ == 0 ? 1 : elastic_last_side_;
    const int sides[2] = {preferred_side, -preferred_side};
    ElasticClearanceScore best_score;
    std::vector<geometry_msgs::PoseStamped> best_candidate;
    int best_side = 0;
    for(double magnitude = elastic_lateral_step_;
        magnitude <= elastic_max_lateral_offset_ + 1e-9;
        magnitude += elastic_lateral_step_)
    {
        for(int side_index = 0; side_index < 2; ++side_index)
        {
            std::vector<geometry_msgs::PoseStamped> candidate = global_plan_;
            candidate[start_index] = current_pose;
            candidate[start_index].header.frame_id =
                costmap_ros_->getGlobalFrameID();
            double along = 0.0;
            for(int index = start_index + 1; index <= end_index; ++index)
            {
                along += std::hypot(
                    costmap_plan[index].pose.position.x -
                        costmap_plan[index - 1].pose.position.x,
                    costmap_plan[index].pose.position.y -
                        costmap_plan[index - 1].pose.position.y);
                const geometry_msgs::PoseStamped& before = costmap_plan[index - 1];
                const geometry_msgs::PoseStamped& after =
                    costmap_plan[std::min(index + 1, end_index)];
                const double tangent_x = after.pose.position.x - before.pose.position.x;
                const double tangent_y = after.pose.position.y - before.pose.position.y;
                const double tangent_length = std::hypot(tangent_x, tangent_y);
                if(!std::isfinite(tangent_length) || tangent_length < 1e-6)
                {
                    candidate.clear();
                    break;
                }
                const double offset = elasticLateralOffset(
                    along, travelled, sides[side_index] * magnitude);
                geometry_msgs::PoseStamped pose = costmap_plan[index];
                pose.pose.position.x += -tangent_y / tangent_length * offset;
                pose.pose.position.y += tangent_x / tangent_length * offset;
                candidate[index] = pose;
            }
            for(int index = start_index + 1;
                !candidate.empty() && index <= end_index; ++index)
            {
                const geometry_msgs::PoseStamped& before = candidate[index - 1];
                const geometry_msgs::PoseStamped& after =
                    candidate[std::min(index + 1, end_index)];
                const double tangent_x = after.pose.position.x - before.pose.position.x;
                const double tangent_y = after.pose.position.y - before.pose.position.y;
                if(!std::isfinite(tangent_x) || !std::isfinite(tangent_y) ||
                   std::hypot(tangent_x, tangent_y) < 1e-6)
                {
                    candidate.clear();
                    break;
                }
                candidate[index].pose.orientation =
                    tf::createQuaternionMsgFromYaw(
                        std::atan2(tangent_y, tangent_x));
            }
            ElasticClearanceScore score;
            score.absolute_offset = magnitude;
            if(!candidate.empty() && elasticCandidateClearance(
                   candidate, start_index, end_index, local_costmap, map_image,
                   score) && elasticCandidateHasMoreClearance(score, best_score))
            {
                best_score = score;
                best_candidate = candidate;
                best_side = sides[side_index];
            }
        }
    }
    if(!best_score.valid ||
       best_score.maximum_cost > maximum_accepted_cost)
    {
        return false;
    }
    elastic_plan_ = best_candidate;
    elastic_active_ = true;
    elastic_end_plan_index_ = end_index;
    elastic_last_side_ = best_side;
    resetControllerState();
    ROS_WARN(
        "cym_planner: activated max-clearance elastic path side=%d "
        "offset=%.3f m horizon=%.3f m max_cost=%u mean_cost=%.1f",
        elastic_last_side_, best_score.absolute_offset, travelled,
        best_score.maximum_cost,
        static_cast<double>(best_score.total_cost) /
            static_cast<double>(best_score.sampled_cells));
    return true;
}

CymPlanner::FootprintBlockage CymPlanner::checkPathBlocked(
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image)
{
    FootprintBlockage path_blockage;
    const PlannerTuning& tuning = activeTuning();
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped current_pose;
    if(!currentCostmapPose(current_pose))
    {
        path_blockage.blocked = true;
        return path_blockage;
    }
    if(body_projection_enabled_)
    {
        path_blockage =
            inspectFootprint(current_pose, local_costmap, map_image, true);
        if(path_blockage.blocked)
        {
            path_blockage.current_footprint_blocked = true;
            clearElasticPlan();
            resetElasticSearch();
            return path_blockage;
        }
    }

    const std::vector<geometry_msgs::PoseStamped>& plan = trackingPlan();
    if(plan.empty())
    {
        path_blockage.blocked = true;
        return path_blockage;
    }
    std::vector<geometry_msgs::PoseStamped> costmap_plan;
    costmap_plan.reserve(plan.size());
    int start_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for(int index = 0; index < static_cast<int>(plan.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_costmap;
        if(!transformPlanPose(plan[index], costmap_frame, pose_costmap) ||
           !poseIsFinite(pose_costmap))
        {
            path_blockage.blocked = true;
            return path_blockage;
        }
        costmap_plan.push_back(pose_costmap);
        const double distance = std::hypot(
            pose_costmap.pose.position.x - current_pose.pose.position.x,
            pose_costmap.pose.position.y - current_pose.pose.position.y);
        if(distance < nearest_distance)
        {
            nearest_distance = distance;
            start_index = index;
        }
    }
    if(elastic_active_ && start_index >= elastic_end_plan_index_)
    {
        clearElasticPlan();
        resetElasticSearch();
        target_index_ = 0;
        return checkPathBlocked(local_costmap, map_image);
    }

    double checked_distance = 0.0;
    double previous_x = 0.0;
    double previous_y = 0.0;
    bool have_previous_point = false;
    for(int index = start_index;
        index < static_cast<int>(costmap_plan.size()); ++index)
    {
        const geometry_msgs::PoseStamped& pose_costmap = costmap_plan[index];
        unsigned int map_x = 0;
        unsigned int map_y = 0;
        if(!local_costmap.worldToMap(
               pose_costmap.pose.position.x,
               pose_costmap.pose.position.y, map_x, map_y))
        {
            path_blockage.blocked = true;
            return path_blockage;
        }
        if(have_previous_point)
        {
            checked_distance += std::hypot(
                pose_costmap.pose.position.x - previous_x,
                pose_costmap.pose.position.y - previous_y);
        }
        previous_x = pose_costmap.pose.position.x;
        previous_y = pose_costmap.pose.position.y;
        have_previous_point = true;
        if(checked_distance > tuning.obstacle_lookahead_distance)
            break;

        if(body_projection_enabled_)
        {
            path_blockage =
                inspectFootprint(pose_costmap, local_costmap, map_image, true);
        }
        else
        {
            path_blockage.blocked = local_costmap.getCost(map_x, map_y) >=
                tuning.obstacle_cost_threshold;
        }
        const bool clearance_limited = body_projection_enabled_ &&
            !path_blockage.blocked &&
            path_blockage.maximum_cost >=
                static_cast<unsigned int>(elastic_activation_cost_);
        if(path_blockage.blocked || clearance_limited)
        {
            const unsigned int maximum_accepted_cost =
                clearance_limited ?
                    static_cast<unsigned int>(elastic_activation_cost_ - 1) :
                    253u;
            if(body_projection_enabled_ && !elastic_active_ &&
               tryActivateElasticPlan(
                   costmap_plan, start_index, current_pose,
                   local_costmap, map_image, maximum_accepted_cost))
            {
                resetElasticSearch();
                return FootprintBlockage();
            }
            if(elastic_active_)
                clearElasticPlan();
            if(clearance_limited)
            {
                path_blockage.blocked = true;
                path_blockage.clearance_limited = true;
                resetElasticSearch();
                ROS_WARN_THROTTLE(
                    1.0,
                    "cym_planner: local forward footprint max cost %u "
                    "has no lower-clearance elastic band; requesting "
                    "global replan",
                    path_blockage.maximum_cost);
                return path_blockage;
            }
            if(body_projection_enabled_ && elastic_enabled_ &&
               !path_blockage.current_footprint_blocked)
            {
                const ros::Time now = ros::Time::now();
                if(elastic_blocked_since_.isZero())
                {
                    elastic_blocked_since_ = now;
                    elastic_search_reference_plan_ = global_plan_;
                }
                if((now - elastic_blocked_since_).toSec() <
                   elastic_search_timeout_)
                {
                    path_blockage.hold = true;
                    ROS_WARN_THROTTLE(
                        1.0,
                        "cym_planner: holding zero while local elastic "
                        "search retries (%.2f s budget)",
                        elastic_search_timeout_);
                    return path_blockage;
                }
            }
            ROS_WARN_THROTTLE(
                1.0, "cym_planner: blocked forward %s; requesting global replan",
                body_projection_enabled_ ? "vehicle footprint" : "path point");
            return path_blockage;
        }
    }
    resetElasticSearch();
    return path_blockage;
}

bool CymPlanner::escapePreviewIsSafe(
    const geometry_msgs::PoseStamped& current_pose,
    const costmap_2d::Costmap2D& local_costmap,
    double direction_world_x,
    double direction_world_y,
    double distance,
    cv::Mat& map_image)
{
    const FootprintBlockage current_blockage =
        inspectFootprint(current_pose, local_costmap, map_image, false);
    const int sample_count = std::max(
        1, static_cast<int>(
            std::ceil(distance / escape_projection_step_)));
    FootprintBlockage preview_blockage;
    for(int sample = 1; sample <= sample_count; ++sample)
    {
        const double sample_distance =
            distance * static_cast<double>(sample) /
            static_cast<double>(sample_count);
        geometry_msgs::PoseStamped preview_pose = current_pose;
        preview_pose.pose.position.x +=
            direction_world_x * sample_distance;
        preview_pose.pose.position.y +=
            direction_world_y * sample_distance;
        preview_blockage =
            inspectFootprint(
                preview_pose, local_costmap, map_image, false);
        if(!escapeIntermediateDoesNotWorsen(
               current_blockage.contact_count,
               preview_blockage.contact_count,
               preview_blockage.blocked,
               preview_blockage.recoverable))
        {
            return false;
        }
    }
    return escapePreviewImproves(
        current_blockage.contact_count,
        preview_blockage.contact_count,
        preview_blockage.blocked,
        preview_blockage.recoverable);
}

bool CymPlanner::computeEscapeCommand(
    const FootprintBlockage& path_blockage,
    const costmap_2d::Costmap2D& local_costmap,
    cv::Mat& map_image,
    geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    if(!body_projection_enabled_ || !path_blockage.recoverable)
        return false;

    geometry_msgs::PoseStamped current_pose;
    if(!currentCostmapPose(current_pose))
    {
        ROS_ERROR_THROTTLE(
            1.0, "cym_planner: escape recovery has no current robot pose");
        return false;
    }

    const ros::Time now = ros::Time::now();
    const double current_yaw = tf::getYaw(current_pose.pose.orientation);
    if(escape_active_)
    {
        const double travelled = std::hypot(
            current_pose.pose.position.x - escape_start_world_x_,
            current_pose.pose.position.y - escape_start_world_y_);
        if(std::abs(normalizeAngle(
               current_yaw - escape_start_world_yaw_)) >
           escape_heading_tolerance_)
        {
            ROS_ERROR(
                "cym_planner: escape recovery heading drift exceeded %.3f rad; "
                "stopping recovery",
                escape_heading_tolerance_);
            resetEscapeRecovery();
            return false;
        }

        const double motion_timeout =
            2.0 * escape_step_distance_ / escape_speed_ + 0.50;
        if((now - escape_motion_started_).toSec() > motion_timeout)
        {
            ROS_ERROR(
                "cym_planner: escape recovery could not move %.3f m within "
                "%.2f s; stopping recovery",
                escape_step_distance_, motion_timeout);
            resetEscapeRecovery();
            return false;
        }

        if(travelled >= escape_step_distance_)
        {
            escape_active_ = false;
            ++escape_attempts_;
            escape_total_distance_ += travelled;
            escape_wait_until_ =
                now + ros::Duration(escape_replan_wait_);
            escape_blocked_since_ = now;
            ROS_WARN(
                "cym_planner: escape step complete attempt=%d/%d "
                "distance=%.3f m total=%.3f/%.3f m; holding %.2f s",
                escape_attempts_, escape_max_attempts_, travelled,
                escape_total_distance_, escape_max_total_distance_,
                escape_replan_wait_);
            return true;
        }

        const double remaining =
            std::max(0.005, escape_step_distance_ - travelled);
        if(!escapePreviewIsSafe(
               current_pose,
               local_costmap,
               escape_direction_world_x_,
               escape_direction_world_y_,
               remaining,
               map_image))
        {
            ROS_ERROR(
                "cym_planner: escape direction is no longer collision-improving; "
                "stopping recovery");
            resetEscapeRecovery();
            return false;
        }

        cmd_vel.linear.x = escape_direction_base_x_ * escape_speed_;
        cmd_vel.linear.y = escape_direction_base_y_ * escape_speed_;
        cmd_vel.angular.z = 0.0;
        return true;
    }

    if(!escape_wait_until_.isZero() && now < escape_wait_until_)
    {
        ROS_WARN_THROTTLE(
            1.0, "cym_planner: holding still for global replan after escape");
        return true;
    }
    escape_wait_until_ = ros::Time(0);

    if(escape_blocked_since_.isZero())
    {
        escape_blocked_since_ = now;
        ROS_WARN(
            "cym_planner: recoverable footprint blockage; holding %.2f s "
            "before a bounded translation",
            escape_blocked_timeout_);
        return true;
    }
    if((now - escape_blocked_since_).toSec() < escape_blocked_timeout_)
        return true;

    if(!escapeBudgetAllows(
           escape_attempts_,
           escape_max_attempts_,
           escape_total_distance_,
           escape_step_distance_,
           escape_max_total_distance_))
    {
        ROS_ERROR(
            "cym_planner: escape recovery limit reached attempts=%d/%d "
            "distance=%.3f/%.3f m; returning control failure",
            escape_attempts_, escape_max_attempts_,
            escape_total_distance_, escape_max_total_distance_);
        resetEscapeRecovery();
        return false;
    }

    const std::vector<geometry_msgs::Point>& footprint =
        costmap_ros_->getRobotFootprint();
    double half_length = 0.0;
    double half_width = 0.0;
    for(const geometry_msgs::Point& point : footprint)
    {
        half_length = std::max(half_length, std::abs(point.x));
        half_width = std::max(half_width, std::abs(point.y));
    }
    const EscapeDirection direction = selectEscapeDirection(
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_yaw,
        path_blockage.contact_world_x,
        path_blockage.contact_world_y,
        half_length,
        half_width);
    if(!direction.valid)
    {
        ROS_ERROR("cym_planner: cannot determine a safe escape side");
        resetEscapeRecovery();
        return false;
    }
    if(!escapePreviewIsSafe(
           current_pose,
           local_costmap,
           direction.world_x,
           direction.world_y,
           escape_step_distance_,
           map_image))
    {
        ROS_ERROR(
            "cym_planner: proposed %.3f m escape from %s contact does not "
            "improve footprint clearance",
            escape_step_distance_,
            escapeContactSideName(direction.contact_side));
        resetEscapeRecovery();
        return false;
    }

    escape_active_ = true;
    escape_motion_started_ = now;
    escape_start_world_x_ = current_pose.pose.position.x;
    escape_start_world_y_ = current_pose.pose.position.y;
    escape_start_world_yaw_ = current_yaw;
    escape_direction_base_x_ = direction.base_x;
    escape_direction_base_y_ = direction.base_y;
    escape_direction_world_x_ = direction.world_x;
    escape_direction_world_y_ = direction.world_y;
    ROS_WARN(
        "cym_planner: escape start side=%s base_direction=(%.0f,%.0f) "
        "step=%.3f m speed=%.3f m/s attempt=%d/%d",
        escapeContactSideName(direction.contact_side),
        escape_direction_base_x_, escape_direction_base_y_,
        escape_step_distance_, escape_speed_,
        escape_attempts_ + 1, escape_max_attempts_);

    cmd_vel.linear.x = escape_direction_base_x_ * escape_speed_;
    cmd_vel.linear.y = escape_direction_base_y_ * escape_speed_;
    cmd_vel.angular.z = 0.0;
    return true;
}

void CymPlanner::publishDebugMap(
    const cv::Mat& map_image,
    const std::string& frame_id) const
{
    if(map_image.empty() || debug_map_pub_.getNumSubscribers() == 0)
        return;
    cv::Mat flipped_image;
    cv::flip(map_image, flipped_image, -1);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    debug_map_pub_.publish(
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                           flipped_image).toImageMsg());
}

void CymPlanner::publishDebugPlan() const
{
    if(debug_plan_pub_.getNumSubscribers() == 0)
        return;

    cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
    for(const geometry_msgs::PoseStamped& plan_pose : global_plan_)
    {
        geometry_msgs::PoseStamped pose_base;
        if(!transformPlanPose(plan_pose, base_link_frame_, pose_base))
            continue;
        const int cv_x = 300 -
            static_cast<int>(pose_base.pose.position.x * 100.0);
        const int cv_y = 300 -
            static_cast<int>(pose_base.pose.position.y * 100.0);
        if(cv_x >= 0 && cv_x < plan_image.cols &&
           cv_y >= 0 && cv_y < plan_image.rows)
        {
            cv::circle(plan_image, cv::Point(cv_x, cv_y), 1,
                       cv::Scalar(255, 0, 255), -1);
        }
    }
    cv::circle(plan_image, cv::Point(300, 300), 15,
               body_projection_enabled_ ?
                   cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0), 2);
    cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300),
             cv::Scalar(0, 255, 0), 1);
    cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555),
             cv::Scalar(0, 255, 0), 1);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = base_link_frame_;
    debug_plan_pub_.publish(
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                           plan_image).toImageMsg());
}

bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    ControlCycleWatchdog cycle_watchdog;
    cmd_vel = geometry_msgs::Twist();
    if(global_plan_.empty())
        return false;
    if(!costmap_ros_ || !costmap_ros_->isCurrent())
    {
        ROS_ERROR_THROTTLE(
            1.0, "cym_planner: local costmap is not current; holding zero");
        return false;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if(!costmap)
        return false;
    costmap_2d::Costmap2D local_costmap;
    {
        costmap_2d::Costmap2D::mutex_t* mutex = costmap->getMutex();
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);
        local_costmap = *costmap;
    }
    const unsigned int size_x = local_costmap.getSizeInCellsX();
    const unsigned int size_y = local_costmap.getSizeInCellsY();
    if(size_x == 0 || size_y == 0 || local_costmap.getResolution() <= 0.0)
        return false;

    cv::Mat map_image;
    const bool debug_map_active =
        debug_images_enabled_ && debug_map_pub_.getNumSubscribers() > 0;
    if(debug_map_active)
    {
        map_image = cv::Mat(
            size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
        for(unsigned int y = 0; y < size_y; ++y)
        {
            for(unsigned int x = 0; x < size_x; ++x)
            {
                const unsigned char cost = local_costmap.getCost(x, y);
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                if(cost == 0)
                    pixel = cv::Vec3b(128, 128, 128);
                else if(cost == 253)
                    pixel = cv::Vec3b(255, 255, 0);
                else if(cost == 254)
                    pixel = cv::Vec3b(0, 0, 0);
                else
                    pixel = cv::Vec3b(255 - cost, 0, 255 - cost);
            }
        }
    }

    const FootprintBlockage path_blockage =
        checkPathBlocked(local_costmap, map_image);
    bool escape_command_valid = false;
    if(escape_enabled_ &&
       path_blockage.blocked &&
       body_projection_enabled_ &&
       path_blockage.recoverable)
    {
        escape_command_valid =
            computeEscapeCommand(
                path_blockage, local_costmap, map_image, cmd_vel);
    }
    if(!map_image.empty())
        map_image.at<cv::Vec3b>(size_y / 2, size_x / 2) =
            cv::Vec3b(0, 255, 0);
    if(debug_images_enabled_)
    {
        publishDebugMap(map_image, costmap_ros_->getGlobalFrameID());
        publishDebugPlan();
    }
    if(path_blockage.blocked)
    {
        if(path_blockage.hold)
            return true;
        return escape_command_valid;
    }
    resetEscapeRecovery();

    geometry_msgs::PoseStamped final_pose;
    if(!transformPlanPose(global_plan_.back(), base_link_frame_, final_pose) ||
       !poseIsFinite(final_pose))
        return false;

    const double final_distance = std::hypot(
        final_pose.pose.position.x, final_pose.pose.position.y);
    if(!std::isfinite(final_distance))
        return false;
    const PlannerTuning& tuning = activeTuning();
    if(!pose_adjusting_ &&
       final_distance < tuning.goal_position_tolerance)
        pose_adjusting_ = true;

    const double motion_scale =
        carry_mode_ ? tuning.carry_speed_scale : 1.0;
    if(pose_adjusting_)
    {
        const double final_yaw = tf::getYaw(final_pose.pose.orientation);
        if(!std::isfinite(final_yaw))
            return false;
        cmd_vel.angular.z = clampValue(
            final_yaw * tuning.final_yaw_gain * motion_scale,
            -tuning.final_yaw_max_vel * motion_scale,
            tuning.final_yaw_max_vel * motion_scale);
        cmd_vel.linear.x = clampValue(
            final_pose.pose.position.x *
                tuning.final_linear_x_gain * motion_scale,
            -tuning.max_vel_x * motion_scale,
            tuning.max_vel_x * motion_scale);
        const bool final_yaw_reached =
            std::abs(final_yaw) < tuning.final_yaw_tolerance;
        const bool final_position_reached =
            final_distance <= tuning.goal_position_tolerance;
        if(final_yaw_reached && final_position_reached)
        {
            cmd_vel = geometry_msgs::Twist();
        }
        if(!commandIsFinite(cmd_vel) ||
           (body_projection_enabled_ &&
             !commandSweepIsSafe(cmd_vel, local_costmap, map_image)))
        {
            cmd_vel = geometry_msgs::Twist();
            return false;
        }
        if(final_yaw_reached && final_position_reached)
        {
            goal_reached_ = true;
            ROS_WARN("cym_planner: goal reached");
        }
        return true;
    }

    geometry_msgs::PoseStamped target_pose;
    if(!selectTargetPose(target_pose) || !poseIsFinite(target_pose))
        return false;

    if(transverse_enabled_)
    {
        // 横向平移模式：车头保持 90°（对齐路径终点朝向），
        // 横向误差（target_pose.y，base_link 系）驱动 linear.y 横移。
        const double final_pose_yaw = tf::getYaw(final_pose.pose.orientation);
        if(!std::isfinite(final_pose_yaw))
            return false;
        double approach_max_vel_y = tuning.max_vel_y;
        if(tuning.approach_decel_distance > 0.0 &&
           std::isfinite(final_distance) &&
           final_distance < tuning.approach_decel_distance)
        {
            const double scale = std::max(
                0.0, final_distance / tuning.approach_decel_distance);
            approach_max_vel_y = std::max(
                tuning.approach_min_vel_x, tuning.max_vel_y * scale);
        }
        const double maximum_lateral_velocity =
            approach_max_vel_y * motion_scale;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = clampValue(
            target_pose.pose.position.y * tuning.lateral_gain *
                motion_scale,
            -maximum_lateral_velocity, maximum_lateral_velocity);
        const double maximum_angular =
            tuning.max_vel_theta * motion_scale;
        cmd_vel.angular.z = clampValue(
            final_pose_yaw * tuning.angular_gain * motion_scale,
            -maximum_angular, maximum_angular);
        if(!commandIsFinite(cmd_vel))
        {
            cmd_vel = geometry_msgs::Twist();
            return false;
        }
        return true;
    }

    const ros::Time control_time = ros::Time::now();
    const double heading_error = std::atan2(
        target_pose.pose.position.y, target_pose.pose.position.x);
    if(!std::isfinite(heading_error))
        return false;
    double heading_derivative = 0.0;
    if(angular_derivative_initialized_)
    {
        const double period =
            (control_time - previous_heading_control_time_).toSec();
        if(std::isfinite(period) && period > 1e-3)
        {
            heading_derivative = clampValue(
                normalizeAngle(heading_error - previous_heading_error_) / period,
                -4.0, 4.0);
        }
    }
    previous_heading_error_ = heading_error;
    previous_heading_control_time_ = control_time;
    angular_derivative_initialized_ = true;
    const double maximum_angular_velocity =
        (elastic_active_ ? std::min(tuning.max_vel_theta, elastic_max_vel_theta_) :
         tuning.max_vel_theta) * motion_scale;
    cmd_vel.angular.z = clampValue(
        (heading_error * tuning.angular_gain +
         heading_derivative * tuning.angular_kd) * motion_scale,
        -maximum_angular_velocity,
        maximum_angular_velocity);

    const double linear_error = target_pose.pose.position.x;
    if(!std::isfinite(linear_error))
        return false;
    double linear_derivative = 0.0;
    if(linear_derivative_initialized_)
    {
        const double period =
            (control_time - previous_linear_control_time_).toSec();
        if(std::isfinite(period) && period > 1e-3)
        {
            linear_derivative = clampValue(
                (linear_error - previous_linear_error_) / period,
                -2.0, 2.0);
        }
    }
    previous_linear_error_ = linear_error;
    previous_linear_control_time_ = control_time;
    linear_derivative_initialized_ = true;
    const double turn_speed_scale = headingSpeedScale(
        heading_error, tuning.heading_slowdown_min_scale);
    double approach_max_vel = tuning.max_vel_x;
    if(tuning.approach_decel_distance > 0.0 &&
       std::isfinite(final_distance) &&
       final_distance < tuning.approach_decel_distance)
    {
        const double scale = std::max(
            0.0, final_distance / tuning.approach_decel_distance);
        approach_max_vel = std::max(
            tuning.approach_min_vel_x, tuning.max_vel_x * scale);
    }
    const double maximum_linear_velocity =
        (elastic_active_ ? std::min(approach_max_vel, elastic_max_vel_x_) :
         approach_max_vel) * motion_scale;
    cmd_vel.linear.x = clampValue(
        (linear_error * tuning.linear_x_gain +
         linear_derivative * tuning.linear_x_kd) *
            motion_scale * turn_speed_scale,
        0.0, maximum_linear_velocity);
    cmd_vel.linear.y = 0.0;
    if(!commandIsFinite(cmd_vel) ||
       (body_projection_enabled_ &&
        !commandSweepIsSafe(cmd_vel, local_costmap, map_image)))
    {
        cmd_vel = geometry_msgs::Twist();
        return false;
    }
    return true;
}

bool CymPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace cym_planner
