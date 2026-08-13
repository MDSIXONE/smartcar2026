#include "cym_planner.h"
#include "cym_planner/velocity_profile.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

double normalizeAngle(double angle)
{
    while(angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while(angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

template <typename T>
void readPlannerParam(const ros::NodeHandle& planner_nh,
                      const ros::NodeHandle& legacy_nh,
                      const std::string& key,
                      T& value,
                      const T& default_value)
{
    if(!planner_nh.getParam(key, value))
    {
        legacy_nh.param<T>(key, value, default_value);
    }
}
}  // namespace

namespace cym_planner
{

CymPlanner::CymPlanner()
    : initialized_(false),
      tf_listener_(nullptr),
      costmap_ros_(nullptr),
      target_index_(0),
      pose_adjusting_(false),
      goal_reached_(false),
      carry_mode_(false),
      main_legacy_previous_linear_error_(0.0),
      main_legacy_previous_control_time_(ros::Time(0)),
      main_legacy_linear_derivative_initialized_(false),
      previous_laser_command_time_(ros::Time(0)),
      laser_blocked_since_(ros::Time(0)),
      laser_blocked_zero_published_(false),
      laser_avoidance_enabled_(false),
      have_scan_(false)
{
}

CymPlanner::~CymPlanner()
{
    delete tf_listener_;
}

void CymPlanner::initialize(std::string name, tf2_ros::Buffer* /* tf */,
                            costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_)
    {
        ROS_WARN("cym_planner: initialize called more than once; ignoring duplicate call");
        return;
    }

    costmap_ros_ = costmap_ros;
    tf_listener_ = new tf::TransformListener();

    ros::NodeHandle planner_nh("~/" + name);
    ros::NodeHandle legacy_nh("~/CymPlanner");
    readPlannerParam(planner_nh, legacy_nh, "base_link_frame", base_link_frame_,
                     std::string("base_link"));
    readPlannerParam(planner_nh, legacy_nh, "scan_topic", scan_topic_,
                     std::string("/scan"));
    readPlannerParam(planner_nh, legacy_nh, "lookahead_distance", lookahead_distance_, 0.50);
    readPlannerParam(planner_nh, legacy_nh, "linear_x_gain", linear_x_gain_, 1.50);
    readPlannerParam(planner_nh, legacy_nh, "angular_gain", angular_gain_, 2.0);
    readPlannerParam(planner_nh, legacy_nh, "max_vel_x", max_vel_x_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "max_vel_theta", max_vel_theta_, 1.5);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_gain", final_yaw_gain_, 2.0);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_max_vel", final_yaw_max_vel_, 1.2);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_tolerance", final_yaw_tolerance_, 0.10);
    readPlannerParam(planner_nh, legacy_nh, "final_linear_x_gain", final_linear_x_gain_, 1.5);
    readPlannerParam(planner_nh, legacy_nh, "goal_position_tolerance",
                     goal_position_tolerance_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "carry_speed_scale", carry_speed_scale_, 1.0);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_target_distance",
                     main_legacy_target_distance_, 0.20);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_linear_x_gain",
                     main_legacy_linear_x_gain_, 10.0);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_linear_x_kd",
                     main_legacy_linear_x_kd_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_angular_gain",
                     main_legacy_angular_gain_, 14.0);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_max_vel_x",
                     main_legacy_max_vel_x_, 14.0);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_max_vel_theta",
                     main_legacy_max_vel_theta_, 20.5);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_final_yaw_gain",
                     main_legacy_final_yaw_gain_, 12.0);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_final_yaw_max_vel",
                     main_legacy_final_yaw_max_vel_, 10.2);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_final_yaw_tolerance",
                     main_legacy_final_yaw_tolerance_, 0.10);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_final_linear_x_gain",
                     main_legacy_final_linear_x_gain_, 1.5);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_goal_position_tolerance",
                     main_legacy_goal_position_tolerance_, 0.05);
    readPlannerParam(planner_nh, legacy_nh,
                     "main_legacy_obstacle_lookahead_distance",
                     main_legacy_obstacle_lookahead_distance_, 0.8);
    readPlannerParam(planner_nh, legacy_nh, "main_legacy_obstacle_cost_threshold",
                     main_legacy_obstacle_cost_threshold_, 253);
    std::string navigation_mode;
    readPlannerParam(planner_nh, legacy_nh, "navigation_mode", navigation_mode,
                     std::string("main_legacy"));
    if(!setNavigationMode(navigation_mode))
    {
        ROS_WARN("[DEBUG:MODE] cym_planner: invalid navigation_mode '%s'; using main_legacy",
                 navigation_mode.c_str());
        laser_avoidance_enabled_.store(false);
    }

    readPlannerParam(planner_nh, legacy_nh, "scan_timeout", scan_timeout_, 0.25);
    readPlannerParam(planner_nh, legacy_nh, "scan_min_range", scan_min_range_, 0.03);
    readPlannerParam(planner_nh, legacy_nh, "scan_max_range", scan_max_range_, 4.0);
    readPlannerParam(planner_nh, legacy_nh, "laser_projection_step",
                     laser_projection_step_, 0.03);
    readPlannerParam(planner_nh, legacy_nh, "safety_margin", safety_margin_, 0.035);
    readPlannerParam(planner_nh, legacy_nh, "braking_deceleration", braking_deceleration_, 3.0);
    readPlannerParam(planner_nh, legacy_nh, "minimum_moving_clearance",
                     minimum_moving_clearance_, 0.10);
    readPlannerParam(planner_nh, legacy_nh, "laser_blocked_grace_period",
                     laser_blocked_grace_period_, 0.25);
    readPlannerParam(planner_nh, legacy_nh, "laser_blocked_hold_max_velocity",
                     laser_blocked_hold_max_velocity_, 0.10);
    readPlannerParam(planner_nh, legacy_nh, "laser_blocked_stop_deceleration",
                     laser_blocked_stop_deceleration_, 0.25);
    readPlannerParam(planner_nh, legacy_nh,
                     "laser_blocked_stop_angular_deceleration",
                     laser_blocked_stop_angular_deceleration_, 1.0);
    readPlannerParam(planner_nh, legacy_nh,
                     "laser_command_linear_rate_limit",
                     laser_command_linear_rate_limit_, 0.50);
    readPlannerParam(planner_nh, legacy_nh,
                     "laser_command_angular_rate_limit",
                     laser_command_angular_rate_limit_, 4.0);
    readPlannerParam(planner_nh, legacy_nh, "reaction_time", reaction_time_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "simulation_time", simulation_time_, 0.30);
    readPlannerParam(planner_nh, legacy_nh, "simulation_step", simulation_step_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "v_samples", v_samples_, 7);
    readPlannerParam(planner_nh, legacy_nh, "w_samples", w_samples_, 9);
    readPlannerParam(planner_nh, legacy_nh, "path_distance_weight",
                     path_distance_weight_, 4.0);
    readPlannerParam(planner_nh, legacy_nh, "heading_weight", heading_weight_, 0.8);
    readPlannerParam(planner_nh, legacy_nh, "clearance_weight", clearance_weight_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "velocity_weight", velocity_weight_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "angular_velocity_weight",
                     angular_velocity_weight_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "minimum_progress_velocity",
                     minimum_progress_velocity_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "minimum_turn_velocity",
                     minimum_turn_velocity_, 0.10);

    lookahead_distance_ = std::max(0.05, lookahead_distance_);
    max_vel_x_ = std::max(0.0, max_vel_x_);
    max_vel_theta_ = std::max(0.0, max_vel_theta_);
    final_yaw_gain_ = std::max(0.0, final_yaw_gain_);
    final_yaw_max_vel_ = std::max(0.0, final_yaw_max_vel_);
    final_yaw_tolerance_ = clampValue(final_yaw_tolerance_, 0.01, kPi);
    final_linear_x_gain_ = std::max(0.0, final_linear_x_gain_);
    goal_position_tolerance_ = std::max(0.01, goal_position_tolerance_);
    carry_speed_scale_ = clampValue(carry_speed_scale_, 0.05, 1.0);
    main_legacy_target_distance_ = std::max(0.01, main_legacy_target_distance_);
    main_legacy_linear_x_gain_ = std::max(0.0, main_legacy_linear_x_gain_);
    main_legacy_linear_x_kd_ = std::max(0.0, main_legacy_linear_x_kd_);
    main_legacy_angular_gain_ = std::max(0.0, main_legacy_angular_gain_);
    main_legacy_max_vel_x_ = std::max(0.0, main_legacy_max_vel_x_);
    main_legacy_max_vel_theta_ = std::max(0.0, main_legacy_max_vel_theta_);
    main_legacy_final_yaw_gain_ = std::max(0.0, main_legacy_final_yaw_gain_);
    main_legacy_final_yaw_max_vel_ = std::max(
        0.0, main_legacy_final_yaw_max_vel_);
    main_legacy_final_yaw_tolerance_ = clampValue(
        main_legacy_final_yaw_tolerance_, 0.01, kPi);
    main_legacy_final_linear_x_gain_ = std::max(
        0.0, main_legacy_final_linear_x_gain_);
    main_legacy_goal_position_tolerance_ = std::max(
        0.01, main_legacy_goal_position_tolerance_);
    main_legacy_obstacle_lookahead_distance_ = std::max(
        0.0, main_legacy_obstacle_lookahead_distance_);
    main_legacy_obstacle_cost_threshold_ = static_cast<int>(
        clampValue(static_cast<double>(main_legacy_obstacle_cost_threshold_),
                   0.0, 255.0));
    scan_timeout_ = std::max(0.05, scan_timeout_);
    scan_min_range_ = std::max(0.0, scan_min_range_);
    scan_max_range_ = std::max(scan_min_range_ + 0.01, scan_max_range_);
    laser_projection_step_ = clampValue(
        laser_projection_step_, 0.01, 0.10);
    safety_margin_ = std::max(0.0, safety_margin_);
    braking_deceleration_ = std::max(0.01, braking_deceleration_);
    minimum_moving_clearance_ = std::max(0.0, minimum_moving_clearance_);
    laser_blocked_grace_period_ = std::max(0.0, laser_blocked_grace_period_);
    laser_blocked_hold_max_velocity_ = std::max(
        0.0, laser_blocked_hold_max_velocity_);
    laser_blocked_stop_deceleration_ = std::max(
        0.01, laser_blocked_stop_deceleration_);
    laser_blocked_stop_angular_deceleration_ = std::max(
        0.01, laser_blocked_stop_angular_deceleration_);
    laser_command_linear_rate_limit_ = std::max(
        0.01, laser_command_linear_rate_limit_);
    laser_command_angular_rate_limit_ = std::max(
        0.01, laser_command_angular_rate_limit_);
    reaction_time_ = std::max(0.0, reaction_time_);
    simulation_time_ = std::max(0.05, simulation_time_);
    simulation_step_ = clampValue(simulation_step_, 0.01, simulation_time_);
    v_samples_ = std::max(2, v_samples_);
    w_samples_ = std::max(3, w_samples_);
    minimum_progress_velocity_ = std::max(0.0, minimum_progress_velocity_);
    minimum_turn_velocity_ = std::max(0.0, minimum_turn_velocity_);

    footprint_min_x_ = std::numeric_limits<double>::infinity();
    footprint_max_x_ = -std::numeric_limits<double>::infinity();
    footprint_min_y_ = std::numeric_limits<double>::infinity();
    footprint_max_y_ = -std::numeric_limits<double>::infinity();
    const std::vector<geometry_msgs::Point>& footprint = costmap_ros_->getRobotFootprint();
    for(const geometry_msgs::Point& point : footprint)
    {
        footprint_min_x_ = std::min(footprint_min_x_, point.x);
        footprint_max_x_ = std::max(footprint_max_x_, point.x);
        footprint_min_y_ = std::min(footprint_min_y_, point.y);
        footprint_max_y_ = std::max(footprint_max_y_, point.y);
    }
    if(footprint.empty())
    {
        ROS_WARN("cym_planner: costmap footprint is empty; using 0.30 m x 0.20 m fallback");
        footprint_min_x_ = -0.15;
        footprint_max_x_ = 0.15;
        footprint_min_y_ = -0.10;
        footprint_max_y_ = 0.10;
    }

    ros::NodeHandle public_nh;
    carry_mode_sub_ = public_nh.subscribe(
        "/sim_task3/carry_mode", 1, &CymPlanner::carryModeCallback, this);
    navigation_mode_sub_ = public_nh.subscribe(
        "/sim_task3/navigation_mode", 1, &CymPlanner::navigationModeCallback, this);
    scan_sub_ = public_nh.subscribe(scan_topic_, 1, &CymPlanner::scanCallback, this);
    laser_points_pub_ = planner_nh.advertise<sensor_msgs::PointCloud2>("laser_points", 1);
    candidate_trajectories_pub_ = planner_nh.advertise<visualization_msgs::MarkerArray>(
        "candidate_trajectories", 1);
    selected_trajectory_pub_ = planner_nh.advertise<visualization_msgs::Marker>(
        "selected_trajectory", 1);
    lookahead_footprint_pub_ = planner_nh.advertise<visualization_msgs::Marker>(
        "lookahead_footprint", 1, true);
    safety_state_pub_ = planner_nh.advertise<std_msgs::String>("safety_state", 1, true);

    initialized_ = true;
    ROS_INFO("[DEBUG:MODE] cym_planner initialized in %s mode; direct laser input=%s, "
             "scan timeout=%.2f s, projected-footprint step=%.2f m",
             navigationModeName(), scan_topic_.c_str(), scan_timeout_,
             laser_projection_step_);
}

bool CymPlanner::setNavigationMode(const std::string& requested_mode)
{
    std::string normalized_mode = requested_mode;
    std::transform(normalized_mode.begin(), normalized_mode.end(), normalized_mode.begin(),
                   [](unsigned char character) { return std::tolower(character); });
    if(normalized_mode == "laser_avoidance" || normalized_mode == "laser" ||
       normalized_mode == "avoidance")
    {
        laser_avoidance_enabled_.store(true);
        return true;
    }
    if(normalized_mode == "main_legacy" || normalized_mode == "main")
    {
        laser_avoidance_enabled_.store(false);
        return true;
    }
    return false;
}

const char* CymPlanner::navigationModeName() const
{
    return laser_avoidance_enabled_.load() ? "laser_avoidance" : "main_legacy";
}

void CymPlanner::navigationModeCallback(const std_msgs::String::ConstPtr& message)
{
    const bool previous_laser_avoidance = laser_avoidance_enabled_.load();
    if(!setNavigationMode(message->data))
    {
        ROS_WARN("[DEBUG:MODE] cym_planner: ignoring unsupported navigation mode '%s'; "
                 "use main_legacy or laser_avoidance",
                 message->data.c_str());
        return;
    }

    if(previous_laser_avoidance != laser_avoidance_enabled_.load())
    {
        resetLaserBrakingState();
        ROS_WARN("[DEBUG:MODE] cym_planner switched to %s mode",
                  navigationModeName());
    }
}

void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
{
    if(carry_mode_ == message->data)
    {
        return;
    }
    carry_mode_ = message->data;
    ROS_INFO("cym_planner carry mode %s; speed scale %.2f",
             carry_mode_ ? "enabled" : "disabled",
             carry_mode_ ? carry_speed_scale_ : 1.0);
}

void CymPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(!initialized_ || scan->header.frame_id.empty())
    {
        return;
    }

    tf::StampedTransform laser_to_base;
    try
    {
        tf_listener_->lookupTransform(base_link_frame_, scan->header.frame_id,
                                      scan->header.stamp, laser_to_base);
    }
    catch(const tf::TransformException&)
    {
        try
        {
            // The laser is rigidly mounted.  During startup the exact scan stamp
            // can precede TF reception by one cycle, while the latest transform is
            // still geometrically correct for this fixed link.
            tf_listener_->lookupTransform(base_link_frame_, scan->header.frame_id,
                                          ros::Time(0), laser_to_base);
        }
        catch(const tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform laser %s to %s: %s",
                              scan->header.frame_id.c_str(), base_link_frame_.c_str(), ex.what());
            return;
        }
    }

    std::vector<LaserPoint> filtered_points;
    filtered_points.reserve(scan->ranges.size());
    const double max_range = std::min(static_cast<double>(scan->range_max), scan_max_range_);
    for(std::size_t index = 0; index < scan->ranges.size(); ++index)
    {
        const double range = scan->ranges[index];
        if(!std::isfinite(range) || range < scan_min_range_ || range > max_range)
        {
            continue;
        }
        const double angle = scan->angle_min + index * scan->angle_increment;
        const tf::Vector3 laser_point(range * std::cos(angle), range * std::sin(angle), 0.0);
        const tf::Vector3 base_point = laser_to_base * laser_point;
        // A 360-degree Gazebo ray sensor can see the robot mesh behind its
        // forward-mounted origin.  Self returns are necessarily inside the
        // physical footprint and must not be treated as external obstacles.
        // Points on or outside the boundary remain core collision input.
        if(base_point.x() > footprint_min_x_ &&
           base_point.x() < footprint_max_x_ &&
           base_point.y() > footprint_min_y_ &&
           base_point.y() < footprint_max_y_)
        {
            continue;
        }
        filtered_points.push_back({base_point.x(), base_point.y()});
    }

    const ros::Time stamp = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        laser_points_ = filtered_points;
        last_scan_stamp_ = stamp;
        have_scan_ = true;
    }
    publishLaserPoints(filtered_points, stamp);
}

bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    global_plan_ = plan;
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    main_legacy_previous_linear_error_ = 0.0;
    main_legacy_previous_control_time_ = ros::Time(0);
    main_legacy_linear_derivative_initialized_ = false;
    // A replacement global plan can arrive while the vehicle is moving.
    // Preserve the last laser command so replanning cannot restart the local
    // controller at an unrelated full-speed sample.  Navigation mode changes
    // still perform the complete reset at their stationary task boundary.
    resetLaserBlockedState();
    return !global_plan_.empty();
}

bool CymPlanner::transformPlanPose(const geometry_msgs::PoseStamped& source,
                                   const std::string& target_frame,
                                   geometry_msgs::PoseStamped& result) const
{
    geometry_msgs::PoseStamped stamped_source = source;
    stamped_source.header.stamp = ros::Time(0);
    try
    {
        tf_listener_->transformPose(target_frame, stamped_source, result);
        return true;
    }
    catch(const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform plan from %s to %s: %s",
                          source.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

bool CymPlanner::selectTargetPose(geometry_msgs::PoseStamped& target_pose,
                                  double lookahead_distance)
{
    for(int index = target_index_; index < static_cast<int>(global_plan_.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_base;
        if(!transformPlanPose(global_plan_[index], base_link_frame_, pose_base))
        {
            return false;
        }

        target_pose = pose_base;
        const double distance = std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);
        if(distance >= lookahead_distance || index == static_cast<int>(global_plan_.size()) - 1)
        {
            target_index_ = index;
            return true;
        }
    }
    return false;
}

bool CymPlanner::isCostmapPathBlocked(double lookahead_distance, int cost_threshold)
{
    if(lookahead_distance <= 0.0)
    {
        return false;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    const int start_index = std::max(0, std::min(
        target_index_, static_cast<int>(global_plan_.size()) - 1));
    bool have_previous_pose = false;
    double previous_x = 0.0;
    double previous_y = 0.0;
    double checked_distance = 0.0;
    geometry_msgs::PoseStamped lookahead_pose;
    bool have_lookahead_pose = false;

    for(int index = start_index; index < static_cast<int>(global_plan_.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_costmap;
        if(!transformPlanPose(global_plan_[index], costmap_frame, pose_costmap))
        {
            return true;
        }

        if(have_previous_pose)
        {
            checked_distance += std::hypot(pose_costmap.pose.position.x - previous_x,
                                           pose_costmap.pose.position.y - previous_y);
        }
        previous_x = pose_costmap.pose.position.x;
        previous_y = pose_costmap.pose.position.y;
        have_previous_pose = true;
        if(checked_distance > lookahead_distance)
        {
            break;
        }

        lookahead_pose = pose_costmap;
        have_lookahead_pose = true;
        unsigned int map_x = 0;
        unsigned int map_y = 0;
        if(costmap->worldToMap(pose_costmap.pose.position.x, pose_costmap.pose.position.y,
                               map_x, map_y) &&
           costmap->getCost(map_x, map_y) >= cost_threshold)
        {
            publishLookaheadFootprint(lookahead_pose, costmap_frame);
            ROS_WARN_THROTTLE(1.0,
                              "cym_planner: auxiliary costmap reports blocked global path; requesting replan");
            return true;
        }
    }

    if(have_lookahead_pose)
    {
        publishLookaheadFootprint(lookahead_pose, costmap_frame);
    }
    return false;
}

bool CymPlanner::checkLaserPathProjection(
    const std::vector<LaserPoint>& points,
    double lookahead_distance,
    bool& projection_blocked)
{
    projection_blocked = false;
    if(lookahead_distance <= 0.0 || global_plan_.empty())
    {
        return true;
    }

    geometry_msgs::PoseStamped projected_pose;
    projected_pose.header.frame_id = base_link_frame_;
    projected_pose.header.stamp = ros::Time::now();
    projected_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    const auto projectionTouchesLaser =
        [this, &points](double robot_x, double robot_y, double robot_yaw,
                        LaserPoint& touching_point)
        {
            const double cos_yaw = std::cos(robot_yaw);
            const double sin_yaw = std::sin(robot_yaw);
            for(const LaserPoint& point : points)
            {
                const double translated_x = point.x - robot_x;
                const double translated_y = point.y - robot_y;
                const double local_x =
                    cos_yaw * translated_x + sin_yaw * translated_y;
                const double local_y =
                    -sin_yaw * translated_x + cos_yaw * translated_y;
                if(local_x >= footprint_min_x_ &&
                   local_x <= footprint_max_x_ &&
                   local_y >= footprint_min_y_ &&
                   local_y <= footprint_max_y_)
                {
                    touching_point = point;
                    return true;
                }
            }
            return false;
        };

    const auto checkProjection =
        [this, &projectionTouchesLaser, &projected_pose,
         &projection_blocked](double robot_x, double robot_y, double robot_yaw)
        {
            projected_pose.pose.position.x = robot_x;
            projected_pose.pose.position.y = robot_y;
            projected_pose.pose.orientation =
                tf::createQuaternionMsgFromYaw(robot_yaw);
            LaserPoint touching_point{0.0, 0.0};
            if(!projectionTouchesLaser(
                   robot_x, robot_y, robot_yaw, touching_point))
            {
                return false;
            }
            projection_blocked = true;
            publishLookaheadFootprint(projected_pose, base_link_frame_);
            ROS_WARN_THROTTLE(
                1.0,
                "cym_planner: laser point (%.3f, %.3f) touches projected "
                "vehicle footprint at (%.3f, %.3f, %.3f); requesting replan",
                touching_point.x, touching_point.y,
                robot_x, robot_y, robot_yaw);
            return true;
        };

    // The first projection is the current physical footprint.  Subsequent
    // projections densely sweep that rectangle along the same global-plan
    // segment followed by the origin/main controller.
    if(checkProjection(0.0, 0.0, 0.0))
    {
        return true;
    }

    const int start_index = std::max(
        0, std::min(target_index_,
                    static_cast<int>(global_plan_.size()) - 1));
    double previous_x = 0.0;
    double previous_y = 0.0;
    double previous_yaw = 0.0;
    double checked_distance = 0.0;
    bool have_forward_projection = false;

    for(int index = start_index;
        index < static_cast<int>(global_plan_.size());
        ++index)
    {
        geometry_msgs::PoseStamped pose_base;
        if(!transformPlanPose(
               global_plan_[index], base_link_frame_, pose_base))
        {
            return false;
        }

        const double current_x = pose_base.pose.position.x;
        const double current_y = pose_base.pose.position.y;
        const double current_yaw = tf::getYaw(pose_base.pose.orientation);
        const double delta_x = current_x - previous_x;
        const double delta_y = current_y - previous_y;
        const double segment_distance = std::hypot(delta_x, delta_y);
        if(segment_distance <= 1e-6)
        {
            previous_yaw = current_yaw;
            continue;
        }

        const double remaining_distance =
            lookahead_distance - checked_distance;
        if(remaining_distance <= 1e-6)
        {
            break;
        }
        const double projected_distance =
            std::min(segment_distance, remaining_distance);
        const int sample_count = std::max(
            1, static_cast<int>(
                   std::ceil(projected_distance / laser_projection_step_)));
        const double yaw_delta =
            normalizeAngle(current_yaw - previous_yaw);

        for(int sample = 1; sample <= sample_count; ++sample)
        {
            const double segment_offset =
                projected_distance * static_cast<double>(sample) /
                static_cast<double>(sample_count);
            const double segment_fraction =
                segment_offset / segment_distance;
            const double projected_x =
                previous_x + delta_x * segment_fraction;
            const double projected_y =
                previous_y + delta_y * segment_fraction;
            const double projected_yaw =
                normalizeAngle(previous_yaw + yaw_delta * segment_fraction);
            have_forward_projection = true;
            projected_pose.pose.position.x = projected_x;
            projected_pose.pose.position.y = projected_y;
            projected_pose.pose.orientation =
                tf::createQuaternionMsgFromYaw(projected_yaw);
            if(checkProjection(projected_x, projected_y, projected_yaw))
            {
                return true;
            }
        }

        checked_distance += projected_distance;
        if(projected_distance + 1e-6 < segment_distance)
        {
            break;
        }
        previous_x = current_x;
        previous_y = current_y;
        previous_yaw = current_yaw;
    }

    if(have_forward_projection)
    {
        publishLookaheadFootprint(projected_pose, base_link_frame_);
    }
    return true;
}

bool CymPlanner::copyFreshLaserPoints(std::vector<LaserPoint>& points,
                                      ros::Time& scan_stamp) const
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if(!have_scan_)
    {
        return false;
    }
    const double age = (ros::Time::now() - last_scan_stamp_).toSec();
    if(age > scan_timeout_)
    {
        return false;
    }
    points = laser_points_;
    scan_stamp = last_scan_stamp_;
    return true;
}

double CymPlanner::clearanceToFootprint(double point_x, double point_y,
                                        double robot_x, double robot_y,
                                        double robot_yaw) const
{
    const double translated_x = point_x - robot_x;
    const double translated_y = point_y - robot_y;
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double local_x = cos_yaw * translated_x + sin_yaw * translated_y;
    const double local_y = -sin_yaw * translated_x + cos_yaw * translated_y;

    const double min_x = footprint_min_x_ - safety_margin_;
    const double max_x = footprint_max_x_ + safety_margin_;
    const double min_y = footprint_min_y_ - safety_margin_;
    const double max_y = footprint_max_y_ + safety_margin_;
    const double dx = std::max(std::max(min_x - local_x, 0.0), local_x - max_x);
    const double dy = std::max(std::max(min_y - local_y, 0.0), local_y - max_y);
    return std::hypot(dx, dy);
}

double CymPlanner::forwardClearance(const std::vector<LaserPoint>& points) const
{
    const double lateral_limit = std::max(std::abs(footprint_min_y_),
                                          std::abs(footprint_max_y_)) + safety_margin_;
    double nearest_clearance = std::numeric_limits<double>::infinity();
    for(const LaserPoint& point : points)
    {
        if(point.x >= footprint_max_x_ && std::abs(point.y) <= lateral_limit)
        {
            nearest_clearance = std::min(nearest_clearance, point.x - footprint_max_x_);
        }
    }
    return nearest_clearance;
}

CymPlanner::CandidateTrajectory CymPlanner::simulateTrajectory(
    double linear_velocity, double angular_velocity, const std::vector<LaserPoint>& points,
    double front_clearance) const
{
    CandidateTrajectory candidate;
    candidate.linear_velocity = linear_velocity;
    candidate.angular_velocity = angular_velocity;
    candidate.clearance = scan_max_range_;
    candidate.score = -std::numeric_limits<double>::infinity();
    candidate.valid = true;

    const double stopping_distance = safety_margin_ + linear_velocity * reaction_time_ +
        linear_velocity * linear_velocity / (2.0 * braking_deceleration_);
    if(linear_velocity > 0.0 && front_clearance < stopping_distance)
    {
        candidate.valid = false;
        return candidate;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(simulation_time_ / simulation_step_)));
    for(int step = 0; step < steps; ++step)
    {
        robot_x += linear_velocity * std::cos(robot_yaw) * simulation_step_;
        robot_y += linear_velocity * std::sin(robot_yaw) * simulation_step_;
        robot_yaw = normalizeAngle(robot_yaw + angular_velocity * simulation_step_);
        candidate.poses.push_back({robot_x, robot_y, robot_yaw});

        for(const LaserPoint& point : points)
        {
            const double clearance = clearanceToFootprint(
                point.x, point.y, robot_x, robot_y, robot_yaw);
            candidate.clearance = std::min(candidate.clearance, clearance);
            if(clearance <= 0.0)
            {
                candidate.valid = false;
                return candidate;
            }
        }
    }
    return candidate;
}

void CymPlanner::rememberLaserCommand(const geometry_msgs::Twist& cmd_vel)
{
    previous_laser_command_ = cmd_vel;
    previous_laser_command_time_ = ros::Time::now();
}

void CymPlanner::resetLaserBlockedState()
{
    laser_blocked_since_ = ros::Time(0);
    laser_blocked_zero_published_ = false;
}

void CymPlanner::resetLaserBrakingState()
{
    previous_laser_command_ = geometry_msgs::Twist();
    previous_laser_command_time_ = ros::Time(0);
    resetLaserBlockedState();
}

bool CymPlanner::computeLaserBlockedCommand(geometry_msgs::Twist& cmd_vel)
{
    if(laser_blocked_zero_published_)
    {
        publishSafetyState(
            "STOP: direct-laser slow stop complete; requesting global replan");
        ROS_WARN_THROTTLE(
            1.0,
            "cym_planner: persistent direct-laser blockage after slow stop; "
            "requesting global replan");
        return false;
    }

    const ros::Time control_time = ros::Time::now();
    if(laser_blocked_since_.isZero())
    {
        laser_blocked_since_ = control_time;
    }

    const double blocked_duration =
        std::max(0.0, (control_time - laser_blocked_since_).toSec());
    const double previous_command_age = previous_laser_command_time_.isZero()
        ? std::numeric_limits<double>::infinity()
        : std::max(
              0.0, (control_time - previous_laser_command_time_).toSec());
    const bool previous_command_is_fresh =
        previous_command_age <= std::max(0.10, laser_blocked_grace_period_);
    const bool previous_command_is_slow =
        std::max(0.0, previous_laser_command_.linear.x) <=
            laser_blocked_hold_max_velocity_ &&
        (std::abs(previous_laser_command_.linear.x) > 0.005 ||
         std::abs(previous_laser_command_.angular.z) > 0.01);

    // A single pitched or noisy scan must not turn into a one-cycle zero Twist.
    // At no more than 0.10 m/s the default 0.25 s grace can move the base by at
    // most 2.5 cm.  Faster commands skip this grace and start ramping down now.
    if(blocked_duration < laser_blocked_grace_period_ &&
       previous_command_is_fresh && previous_command_is_slow)
    {
        cmd_vel = previous_laser_command_;
        rememberLaserCommand(cmd_vel);
        publishSafetyState(
            "HOLDING: transient direct-laser blockage; keeping prior low-speed command");
        return true;
    }

    double control_period = 0.05;
    if(previous_command_is_fresh)
    {
        control_period = clampValue(previous_command_age, 0.01, 0.10);
    }
    const double next_linear_velocity = approachVelocity(
        std::max(0.0, previous_laser_command_.linear.x),
        0.0, laser_blocked_stop_deceleration_, control_period);
    const double next_angular_velocity = approachVelocity(
        previous_laser_command_.angular.z,
        0.0, laser_blocked_stop_angular_deceleration_, control_period);
    constexpr double kStoppedLinearVelocity = 0.005;
    constexpr double kStoppedAngularVelocity = 0.01;
    if(next_linear_velocity <= kStoppedLinearVelocity &&
       std::abs(next_angular_velocity) <= kStoppedAngularVelocity)
    {
        cmd_vel = geometry_msgs::Twist();
        rememberLaserCommand(cmd_vel);
        laser_blocked_zero_published_ = true;
        publishSafetyState(
            "DECELERATING: persistent direct-laser blockage reached zero");
        return true;
    }

    cmd_vel.linear.x = next_linear_velocity;
    cmd_vel.angular.z = next_angular_velocity;
    rememberLaserCommand(cmd_vel);
    std::ostringstream state;
    state.setf(std::ios::fixed);
    state.precision(2);
    state << "DECELERATING: persistent direct-laser blockage v="
          << next_linear_velocity << " w=" << next_angular_velocity;
    publishSafetyState(state.str());
    return true;
}

void CymPlanner::computeSmoothedLaserCommand(
    const CandidateTrajectory& selected,
    const std::vector<LaserPoint>& laser_points,
    double front_clearance,
    geometry_msgs::Twist& cmd_vel)
{
    const ros::Time control_time = ros::Time::now();
    const double previous_command_age = previous_laser_command_time_.isZero()
        ? std::numeric_limits<double>::infinity()
        : std::max(
              0.0, (control_time - previous_laser_command_time_).toSec());
    const bool previous_command_is_fresh =
        previous_command_age <= std::max(0.20, scan_timeout_);
    const double control_period = previous_command_is_fresh
        ? clampValue(previous_command_age, 0.01, 0.10)
        : 0.05;
    const double current_linear_velocity = previous_command_is_fresh
        ? previous_laser_command_.linear.x : 0.0;
    const double current_angular_velocity = previous_command_is_fresh
        ? previous_laser_command_.angular.z : 0.0;
    const double smoothed_linear_velocity = approachVelocity(
        current_linear_velocity, selected.linear_velocity,
        laser_command_linear_rate_limit_, control_period);
    const double smoothed_angular_velocity = approachVelocity(
        current_angular_velocity, selected.angular_velocity,
        laser_command_angular_rate_limit_, control_period);

    // Preserve continuity on both axes when that interpolated arc is safe.  If
    // it is not, retain one smoothed axis at a time before falling back to the
    // original laser-validated target.  Smoothing therefore cannot manufacture
    // an unchecked collision trajectory.
    const double trial_linear_velocity[] = {
        smoothed_linear_velocity,
        selected.linear_velocity,
        smoothed_linear_velocity,
        selected.linear_velocity,
    };
    const double trial_angular_velocity[] = {
        smoothed_angular_velocity,
        smoothed_angular_velocity,
        selected.angular_velocity,
        selected.angular_velocity,
    };
    for(std::size_t index = 0;
        index < sizeof(trial_linear_velocity) / sizeof(double);
        ++index)
    {
        const CandidateTrajectory trial = simulateTrajectory(
            trial_linear_velocity[index], trial_angular_velocity[index],
            laser_points, front_clearance);
        if(!trial.valid)
        {
            continue;
        }
        cmd_vel.linear.x = trial_linear_velocity[index];
        cmd_vel.angular.z = trial_angular_velocity[index];
        rememberLaserCommand(cmd_vel);
        return;
    }

    // Defensive fallback: `selected` was already validated while constructing
    // the candidate set.
    cmd_vel.linear.x = selected.linear_velocity;
    cmd_vel.angular.z = selected.angular_velocity;
    rememberLaserCommand(cmd_vel);
}

void CymPlanner::publishLaserPoints(const std::vector<LaserPoint>& points,
                                    const ros::Time& stamp) const
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = base_link_frame_;
    cloud.header.stamp = stamp;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for(const LaserPoint& point : points)
    {
        *x_iterator = static_cast<float>(point.x);
        *y_iterator = static_cast<float>(point.y);
        *z_iterator = 0.03F;
        ++x_iterator;
        ++y_iterator;
        ++z_iterator;
    }
    laser_points_pub_.publish(cloud);
}

void CymPlanner::publishTrajectoryDebug(
    const std::vector<CandidateTrajectory>& candidates, int selected_index) const
{
    const ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = base_link_frame_;
    clear_marker.header.stamp = now;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for(std::size_t index = 0; index < candidates.size(); ++index)
    {
        const CandidateTrajectory& candidate = candidates[index];
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_link_frame_;
        marker.header.stamp = now;
        marker.ns = "cym_planner_candidates";
        marker.id = static_cast<int>(index);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.008;
        marker.color.a = candidate.valid ? 0.35 : 0.25;
        marker.color.r = candidate.valid ? 0.15F : 1.0F;
        marker.color.g = candidate.valid ? 0.65F : 0.10F;
        marker.color.b = candidate.valid ? 1.0F : 0.10F;
        marker.lifetime = ros::Duration(0.25);
        for(const TrajectoryPose& pose : candidate.poses)
        {
            geometry_msgs::Point point;
            point.x = pose.x;
            point.y = pose.y;
            point.z = 0.04;
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    candidate_trajectories_pub_.publish(marker_array);

    visualization_msgs::Marker selected_marker;
    selected_marker.header.frame_id = base_link_frame_;
    selected_marker.header.stamp = now;
    selected_marker.ns = "cym_planner_selected";
    selected_marker.id = 0;
    selected_marker.type = visualization_msgs::Marker::LINE_STRIP;
    selected_marker.scale.x = 0.025;
    selected_marker.color.r = 0.0F;
    selected_marker.color.g = 1.0F;
    selected_marker.color.b = 0.10F;
    selected_marker.color.a = 1.0F;
    selected_marker.lifetime = ros::Duration(0.25);
    if(selected_index < 0)
    {
        selected_marker.action = visualization_msgs::Marker::DELETE;
    }
    else
    {
        selected_marker.action = visualization_msgs::Marker::ADD;
        for(const TrajectoryPose& pose : candidates[selected_index].poses)
        {
            geometry_msgs::Point point;
            point.x = pose.x;
            point.y = pose.y;
            point.z = 0.05;
            selected_marker.points.push_back(point);
        }
    }
    selected_trajectory_pub_.publish(selected_marker);
}

void CymPlanner::publishLookaheadFootprint(const geometry_msgs::PoseStamped& lookahead_pose,
                                           const std::string& costmap_frame) const
{
    const std::vector<geometry_msgs::Point>& footprint = costmap_ros_->getRobotFootprint();
    if(footprint.empty())
    {
        return;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = costmap_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cym_planner_costmap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = lookahead_pose.pose;
    marker.pose.position.z += 0.03;
    marker.scale.x = 0.02;
    marker.color.r = 0.05F;
    marker.color.g = 0.95F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker.points = footprint;
    marker.points.push_back(footprint.front());
    lookahead_footprint_pub_.publish(marker);
}

void CymPlanner::publishSafetyState(const std::string& state) const
{
    std_msgs::String message;
    message.data = state;
    safety_state_pub_.publish(message);
}

bool CymPlanner::computeMainLegacyCommands(
    geometry_msgs::Twist& cmd_vel,
    bool use_laser_projection)
{
    cmd_vel = geometry_msgs::Twist();

    if(use_laser_projection)
    {
        std::vector<LaserPoint> laser_points;
        ros::Time scan_stamp;
        if(!copyFreshLaserPoints(laser_points, scan_stamp))
        {
            publishSafetyState(
                "STOP: laser projection scan unavailable or stale");
            ROS_WARN_THROTTLE(
                1.0,
                "cym_planner: refusing projected-footprint path following "
                "without a fresh %s scan",
                scan_topic_.c_str());
            return false;
        }

        bool projection_blocked = false;
        if(!checkLaserPathProjection(
               laser_points,
               main_legacy_obstacle_lookahead_distance_,
               projection_blocked))
        {
            publishSafetyState(
                "STOP: cannot transform laser vehicle projection");
            return false;
        }
        if(projection_blocked)
        {
            publishSafetyState(
                "STOP: laser touches projected vehicle footprint");
            return false;
        }
    }
    else
    {
        // The pre-pickup phase stays byte-for-byte equivalent in behaviour to
        // origin/main: the rolling costmap hands blocked paths back to
        // move_base/global_planner for replanning.
        if(isCostmapPathBlocked(main_legacy_obstacle_lookahead_distance_,
                                main_legacy_obstacle_cost_threshold_))
        {
            publishSafetyState(
                "STOP: main_legacy costmap requests global replan");
            return false;
        }
    }

    geometry_msgs::PoseStamped final_pose;
    if(!transformPlanPose(global_plan_.back(), base_link_frame_, final_pose))
    {
        publishSafetyState("STOP: main_legacy cannot transform final plan pose");
        return false;
    }
    const double final_distance = std::hypot(
        final_pose.pose.position.x, final_pose.pose.position.y);
    if(!pose_adjusting_ &&
       final_distance < main_legacy_goal_position_tolerance_)
    {
        pose_adjusting_ = true;
    }

    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    if(pose_adjusting_)
    {
        const double final_yaw = tf::getYaw(final_pose.pose.orientation);
        cmd_vel.angular.z = clampValue(
            final_yaw * main_legacy_final_yaw_gain_ * motion_scale,
            -main_legacy_final_yaw_max_vel_ * motion_scale,
            main_legacy_final_yaw_max_vel_ * motion_scale);
        cmd_vel.linear.x = final_pose.pose.position.x *
            main_legacy_final_linear_x_gain_ * motion_scale;
        if(std::abs(final_yaw) < main_legacy_final_yaw_tolerance_)
        {
            goal_reached_ = true;
            cmd_vel = geometry_msgs::Twist();
            publishSafetyState("GOAL_REACHED");
        }
        else
        {
            publishSafetyState("ACTIVE: origin/main final-pose alignment");
        }
        return true;
    }

    geometry_msgs::PoseStamped target_pose;
    if(!selectTargetPose(target_pose, main_legacy_target_distance_))
    {
        publishSafetyState("STOP: main_legacy cannot select local path target");
        return false;
    }

    const double heading_error = std::atan2(
        target_pose.pose.position.y, target_pose.pose.position.x);
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = clampValue(
        heading_error * main_legacy_angular_gain_ * motion_scale,
        -main_legacy_max_vel_theta_ * motion_scale,
        main_legacy_max_vel_theta_ * motion_scale);
    const double heading_speed_scale = std::max(
        0.25, std::cos(std::min(std::abs(heading_error), kPi / 2.0)));

    const double linear_error = target_pose.pose.position.x;
    const ros::Time control_time = ros::Time::now();
    double linear_error_derivative = 0.0;
    if(main_legacy_linear_derivative_initialized_)
    {
        const double control_period =
            (control_time - main_legacy_previous_control_time_).toSec();
        if(control_period > 1e-3)
        {
            linear_error_derivative = clampValue(
                (linear_error - main_legacy_previous_linear_error_) /
                    control_period,
                -2.0, 2.0);
        }
    }
    main_legacy_previous_linear_error_ = linear_error;
    main_legacy_previous_control_time_ = control_time;
    main_legacy_linear_derivative_initialized_ = true;

    const double linear_control =
        (linear_error * main_legacy_linear_x_gain_ +
         linear_error_derivative * main_legacy_linear_x_kd_) * motion_scale;
    cmd_vel.linear.x = std::max(
        0.0,
        std::min(linear_control, main_legacy_max_vel_x_ * motion_scale) *
            heading_speed_scale);
    publishTrajectoryDebug(std::vector<CandidateTrajectory>(), -1);
    publishSafetyState(use_laser_projection
        ? "ACTIVE: origin/main CymPlanner with laser vehicle projection"
        : "ACTIVE: origin/main CymPlanner with local-costmap replanning");
    return true;
}

bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    if(!initialized_ || global_plan_.empty())
    {
        publishSafetyState("STOP: empty global plan");
        return false;
    }

    // Both task phases now use exactly the origin/main line-following control
    // law.  The post-pickup mode changes only the obstacle predicate: direct
    // laser points are tested against the vehicle footprint swept along the
    // lookahead path, and a touch returns false so move_base replans.
    return computeMainLegacyCommands(
        cmd_vel, laser_avoidance_enabled_.load());

    std::vector<LaserPoint> laser_points;
    ros::Time scan_stamp;
    if(!copyFreshLaserPoints(laser_points, scan_stamp))
    {
        publishSafetyState("STOP: laser scan unavailable or stale");
        ROS_WARN_THROTTLE(1.0, "cym_planner: refusing to move without a fresh %s scan",
                          scan_topic_.c_str());
        return false;
    }
    if(laser_points.empty())
    {
        publishSafetyState("STOP: laser scan has no valid points");
        return false;
    }

    geometry_msgs::PoseStamped final_pose;
    if(!transformPlanPose(global_plan_.back(), base_link_frame_, final_pose))
    {
        publishSafetyState("STOP: cannot transform final plan pose");
        return false;
    }
    const double final_distance = std::hypot(final_pose.pose.position.x, final_pose.pose.position.y);
    if(final_distance < goal_position_tolerance_)
    {
        pose_adjusting_ = true;
    }

    if(pose_adjusting_ &&
       std::abs(tf::getYaw(final_pose.pose.orientation)) < final_yaw_tolerance_)
    {
        goal_reached_ = true;
        rememberLaserCommand(cmd_vel);
        publishSafetyState("GOAL_REACHED");
        return true;
    }

    geometry_msgs::PoseStamped target_pose;
    double desired_linear_velocity = 0.0;
    double desired_angular_velocity = 0.0;
    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    if(pose_adjusting_)
    {
        target_pose = final_pose;
        // The positional tolerance has already been met.  Keep the vehicle
        // stationary and let the laser-validated trajectories solve only the
        // final orientation; advancing here can turn a small pose error into a
        // large arc around the goal.
        desired_linear_velocity = 0.0;
        const double final_yaw_error = tf::getYaw(final_pose.pose.orientation);
        const double proportional_yaw_velocity =
            final_yaw_error * final_yaw_gain_ * motion_scale;
        // A rollout scores its pose at simulation_time_.  Limiting the command
        // to the angle that can be completed inside that horizon prevents a
        // high legacy yaw gain from overshooting the terminal orientation.
        const double horizon_yaw_velocity =
            final_yaw_error / simulation_time_ * motion_scale;
        const double yaw_velocity_limit = std::min(
            final_yaw_max_vel_ * motion_scale, std::abs(horizon_yaw_velocity));
        desired_angular_velocity = clampValue(
            proportional_yaw_velocity, -yaw_velocity_limit, yaw_velocity_limit);
    }
    else
    {
        if(!selectTargetPose(target_pose, lookahead_distance_))
        {
            publishSafetyState("STOP: cannot select local path target");
            return false;
        }
        const double heading_error = std::atan2(target_pose.pose.position.y,
                                                target_pose.pose.position.x);
        const double nominal_linear_velocity = clampValue(
            target_pose.pose.position.x * linear_x_gain_ * motion_scale,
            0.0, max_vel_x_ * motion_scale);
        // Do not enter a tight bend at straight-line speed.  Squared cosine
        // keeps gentle curves fast while giving the angular controller time to
        // turn the complete physical footprint before it reaches the wall.
        const double heading_cosine = std::cos(heading_error);
        const double turn_speed_scale = clampValue(
            heading_cosine * heading_cosine, 0.15, 1.0);
        desired_linear_velocity = nominal_linear_velocity * turn_speed_scale;
        desired_angular_velocity = clampValue(
            heading_error * angular_gain_ * motion_scale,
            -max_vel_theta_ * motion_scale, max_vel_theta_ * motion_scale);
    }

    const double max_angular_velocity = pose_adjusting_
        ? final_yaw_max_vel_ * motion_scale : max_vel_theta_ * motion_scale;
    const double front_clearance = forwardClearance(laser_points);
    std::vector<double> angular_candidates;
    angular_candidates.reserve(static_cast<std::size_t>(w_samples_ + 6));
    const auto append_angular_candidate =
        [&angular_candidates, max_angular_velocity](double value)
        {
            const double bounded = clampValue(
                value, -max_angular_velocity, max_angular_velocity);
            for(const double existing : angular_candidates)
            {
                if(std::abs(existing - bounded) < 1e-6)
                {
                    return;
                }
            }
            angular_candidates.push_back(bounded);
        };
    for(int w_index = 0; w_index < w_samples_; ++w_index)
    {
        const double center = 0.5 * static_cast<double>(w_samples_ - 1);
        const double angular_offset = (static_cast<double>(w_index) - center) /
            std::max(1.0, center) * max_angular_velocity;
        append_angular_candidate(desired_angular_velocity + angular_offset);
    }
    // Near a wall, a full desired turn can collide over the rollout horizon
    // even though a slower turn in the same direction is safe.  Always sample
    // an idle command plus fractional target-directed turns; otherwise the
    // shifted angular grid can omit w=0 and every candidate is rejected.
    append_angular_candidate(0.0);
    if(std::abs(desired_angular_velocity) > 1e-6)
    {
        append_angular_candidate(std::copysign(
            minimum_turn_velocity_, desired_angular_velocity));
    }
    append_angular_candidate(desired_angular_velocity * 0.25);
    append_angular_candidate(desired_angular_velocity * 0.50);
    append_angular_candidate(desired_angular_velocity * 0.75);
    append_angular_candidate(desired_angular_velocity);

    std::vector<CandidateTrajectory> candidates;
    candidates.reserve(
        static_cast<std::size_t>(v_samples_) * angular_candidates.size());
    int selected_index = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    // In path-following mode, heading points at the local path target.  Once
    // position is within tolerance, the target is instead the goal's final
    // orientation.  Using atan2(y, x) in this branch tends to zero and makes
    // the scorer prefer w = 0, which is why the previous rollout stopped
    // turning at the destination.
    const double target_heading = pose_adjusting_
        ? tf::getYaw(target_pose.pose.orientation)
        : std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);

    for(int v_index = 0; v_index < v_samples_; ++v_index)
    {
        const double fraction = static_cast<double>(v_index) /
            static_cast<double>(v_samples_ - 1);
        const double candidate_linear_velocity = desired_linear_velocity * fraction;
        for(const double candidate_angular_velocity : angular_candidates)
        {
            CandidateTrajectory candidate = simulateTrajectory(
                candidate_linear_velocity, candidate_angular_velocity, laser_points,
                front_clearance);
            if(candidate.valid &&
               candidate.linear_velocity >= minimum_progress_velocity_ &&
               candidate.clearance < minimum_moving_clearance_)
            {
                // Merely avoiding geometric overlap is not enough for the
                // dynamic cones.  A near-zero-clearance arc lets the physical
                // body touch and climb a cone before the next scan arrives.
                candidate.valid = false;
            }
            if(candidate.valid && !candidate.poses.empty())
            {
                const TrajectoryPose& end_pose = candidate.poses.back();
                const double path_error = std::hypot(
                    end_pose.x - target_pose.pose.position.x,
                    end_pose.y - target_pose.pose.position.y);
                const double heading_error = std::abs(normalizeAngle(target_heading - end_pose.yaw));
                const double normalized_speed = desired_linear_velocity > 1e-4
                    ? candidate.linear_velocity / desired_linear_velocity : 0.0;
                candidate.score =
                    -path_distance_weight_ * path_error
                    -heading_weight_ * heading_error
                    +clearance_weight_ * std::min(candidate.clearance, scan_max_range_)
                    +velocity_weight_ * normalized_speed
                    -angular_velocity_weight_ * std::abs(
                        candidate.angular_velocity - desired_angular_velocity);
                if(candidate.score > best_score)
                {
                    best_score = candidate.score;
                    selected_index = static_cast<int>(candidates.size());
                }
            }
            candidates.push_back(candidate);
        }
    }

    if(selected_index < 0)
    {
        publishTrajectoryDebug(candidates, selected_index);
        ROS_WARN_THROTTLE(1.0,
                          "cym_planner: laser point cloud rejects every rollout; "
                          "applying continuous slow stop");
        return computeLaserBlockedCommand(cmd_vel);
    }

    if(!pose_adjusting_)
    {
        int best_safe_forward_index = -1;
        int best_goal_directed_turn_index = -1;
        double best_safe_forward_score = -std::numeric_limits<double>::infinity();
        double smallest_turn_heading_error = std::numeric_limits<double>::infinity();
        double best_turn_score = -std::numeric_limits<double>::infinity();
        const double initial_heading_error = std::abs(normalizeAngle(target_heading));

        for(std::size_t index = 0; index < candidates.size(); ++index)
        {
            const CandidateTrajectory& candidate = candidates[index];
            if(candidate.valid &&
               candidate.linear_velocity >= minimum_progress_velocity_)
            {
                if(candidate.score > best_safe_forward_score)
                {
                    best_safe_forward_score = candidate.score;
                    best_safe_forward_index = static_cast<int>(index);
                }
            }

            // At a pickup bay the global target can be behind the vehicle while
            // the shelf blocks every *forward* rollout.  That is not a lidar
            // deadlock: a collision-free turn that reduces the heading error is
            // the required first step before forward progress becomes possible.
            // Never accept an arbitrary stationary spin here; it must make a
            // measurable improvement toward the current global-plan target.
            if(candidate.valid && !candidate.poses.empty() &&
               candidate.linear_velocity < minimum_progress_velocity_ &&
               std::abs(candidate.angular_velocity) >= minimum_turn_velocity_)
            {
                const double end_heading_error = std::abs(normalizeAngle(
                    target_heading - candidate.poses.back().yaw));
                constexpr double kMinimumHeadingImprovement = 0.01;
                if(end_heading_error < initial_heading_error - kMinimumHeadingImprovement &&
                   (end_heading_error < smallest_turn_heading_error ||
                    (end_heading_error == smallest_turn_heading_error &&
                     candidate.score > best_turn_score)))
                {
                    smallest_turn_heading_error = end_heading_error;
                    best_turn_score = candidate.score;
                    best_goal_directed_turn_index = static_cast<int>(index);
                }
            }
        }

        const CandidateTrajectory& selected = candidates[selected_index];
        const bool selected_command_is_idle =
            selected.linear_velocity < minimum_progress_velocity_ &&
            std::abs(selected.angular_velocity) < minimum_turn_velocity_;
        if(best_safe_forward_index < 0)
        {
            if(best_goal_directed_turn_index < 0)
            {
                publishTrajectoryDebug(candidates, selected_index);
                ROS_WARN_THROTTLE(1.0,
                                  "cym_planner: no forward lidar rollout; applying transient hold or slow stop");
                return computeLaserBlockedCommand(cmd_vel);
            }

            selected_index = best_goal_directed_turn_index;
            ROS_INFO_THROTTLE(1.0,
                              "cym_planner: no forward lidar rollout; rotating toward the global-plan target");
        }
        else if(selected_command_is_idle)
        {
            // The scorer may tie on an idle command.  If a laser-safe forward
            // rollout exists, choose it so navigation cannot silently stall.
            selected_index = best_safe_forward_index;
        }
    }

    resetLaserBlockedState();
    publishTrajectoryDebug(candidates, selected_index);
    const CandidateTrajectory& selected = candidates[selected_index];

    computeSmoothedLaserCommand(
        selected, laser_points, front_clearance, cmd_vel);
    std::ostringstream state;
    state.setf(std::ios::fixed);
    state.precision(2);
    state << "ACTIVE: direct laser rollout selected"
          << " v=" << selected.linear_velocity
          << " w=" << selected.angular_velocity
          << " command_v=" << cmd_vel.linear.x
          << " command_w=" << cmd_vel.angular.z
          << " clearance=" << selected.clearance;
    publishSafetyState(state.str());
    return true;
}

bool CymPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace cym_planner
