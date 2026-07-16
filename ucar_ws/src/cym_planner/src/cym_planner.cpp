#include "cym_planner.h"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace cym_planner
{
namespace
{
tf::TransformListener* tf_listener_ = nullptr;
costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
std::string base_link_frame_;
double linear_x_gain_;
double linear_x_kd_;
double linear_y_gain_;
double linear_y_kd_;
double angular_gain_;
double angular_kd_;
double max_vel_x_;
double max_vel_y_;
double max_vel_theta_;
double final_yaw_gain_;
double final_yaw_max_vel_;
double final_yaw_tolerance_;
double final_linear_x_gain_;
double obstacle_lookahead_distance_;
int obstacle_cost_threshold_;
double previous_linear_error_;
ros::Time previous_control_time_;
bool linear_derivative_initialized_;
double previous_lateral_error_;
ros::Time previous_lateral_control_time_;
bool lateral_derivative_initialized_;
double previous_heading_error_;
ros::Time previous_heading_control_time_;
bool angular_derivative_initialized_;
std::vector<geometry_msgs::PoseStamped> global_plan_;
int target_index_;
bool pose_adjusting_;
bool goal_reached_;
bool holonomic_mode_;
double task_max_vel_;

// move_base versions can pass either "CymPlanner" or the fully-qualified
// plugin name to initialize().  Keep the configuration namespace explicit so
// the selected YAML is used in both cases.
template <typename T>
bool readPlannerParam(
    ros::NodeHandle& runtime_nh,
    ros::NodeHandle& canonical_nh,
    ros::NodeHandle& legacy_nh,
    const std::string& key,
    T& value)
{
    return runtime_nh.getParam(key, value) ||
           canonical_nh.getParam(key, value) ||
           legacy_nh.getParam(key, value);
}
}

CymPlanner::CymPlanner() = default;
CymPlanner::~CymPlanner() = default;

void CymPlanner::initialize(
    std::string name, tf2_ros::Buffer* /*tf*/,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    delete tf_listener_;
    tf_listener_ = new tf::TransformListener();
    costmap_ros_ = costmap_ros;

    ros::NodeHandle planner_nh("~/" + name);
    ros::NodeHandle canonical_nh("~/cym_planner/CymPlanner");
    ros::NodeHandle legacy_nh("~/CymPlanner");
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "base_link_frame", base_link_frame_))
        base_link_frame_ = "base_link";
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "linear_x_gain", linear_x_gain_))
        linear_x_gain_ = 1.5;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "linear_x_kd", linear_x_kd_))
        linear_x_kd_ = 0.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "linear_y_gain", linear_y_gain_))
        linear_y_gain_ = 1.5;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "linear_y_kd", linear_y_kd_))
        linear_y_kd_ = 0.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "angular_gain", angular_gain_))
        angular_gain_ = 2.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "angular_kd", angular_kd_))
        angular_kd_ = 0.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "max_vel_x", max_vel_x_))
        max_vel_x_ = 0.2;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "max_vel_y", max_vel_y_))
        max_vel_y_ = 0.2;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "max_vel_theta", max_vel_theta_))
        max_vel_theta_ = 0.5;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "final_yaw_gain", final_yaw_gain_))
        final_yaw_gain_ = 1.5;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "final_yaw_max_vel", final_yaw_max_vel_))
        final_yaw_max_vel_ = 0.4;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "final_yaw_tolerance", final_yaw_tolerance_))
        final_yaw_tolerance_ = 0.10;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "final_linear_x_gain", final_linear_x_gain_))
        final_linear_x_gain_ = 1.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "obstacle_lookahead_distance", obstacle_lookahead_distance_))
        obstacle_lookahead_distance_ = 0.25;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "obstacle_cost_threshold", obstacle_cost_threshold_))
        obstacle_cost_threshold_ = 253;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "carry_speed_scale", carry_speed_scale_))
        carry_speed_scale_ = 1.0;
    if (!readPlannerParam(planner_nh, canonical_nh, legacy_nh,
                          "holonomic_mode", holonomic_mode_))
        holonomic_mode_ = false;

    obstacle_lookahead_distance_ = std::max(0.0, obstacle_lookahead_distance_);
    obstacle_cost_threshold_ = std::max(0, std::min(255, obstacle_cost_threshold_));
    final_yaw_tolerance_ = std::max(0.01, std::min(M_PI, final_yaw_tolerance_));
    carry_speed_scale_ = std::max(0.05, std::min(1.0, carry_speed_scale_));
    carry_mode_ = false;
    ros::NodeHandle public_nh;
    carry_mode_sub_ = public_nh.subscribe(
        "/sim_task3/carry_mode", 1, &CymPlanner::carryModeCallback, this);
    previous_linear_error_ = 0.0;
    previous_control_time_ = ros::Time(0);
    linear_derivative_initialized_ = false;
    previous_lateral_error_ = 0.0;
    previous_lateral_control_time_ = ros::Time(0);
    lateral_derivative_initialized_ = false;
    previous_heading_error_ = 0.0;
    previous_heading_control_time_ = ros::Time(0);
    angular_derivative_initialized_ = false;

    ROS_INFO("cym_planner initialized: plugin=%s, holonomic=%s, max_vel_x=%.2f, max_vel_y=%.2f, max_vel_theta=%.2f, angular_p=%.2f, angular_d=%.2f, local obstacle lookahead=%.2f",
             name.c_str(), holonomic_mode_ ? "true" : "false", max_vel_x_, max_vel_y_, max_vel_theta_, angular_gain_, angular_kd_,
             obstacle_lookahead_distance_);
}

void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
{
    carry_mode_ = message->data;
}

bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    // The task node changes this parameter before it sends a new goal.  Read
    // it for every plan so normal and holonomic navigation can coexist in one
    // move_base session without a node restart.
    ros::NodeHandle canonical_nh("~/cym_planner/CymPlanner");
    bool requested_holonomic_mode = holonomic_mode_;
    if (canonical_nh.getParam("holonomic_mode", requested_holonomic_mode) &&
        requested_holonomic_mode != holonomic_mode_)
    {
        holonomic_mode_ = requested_holonomic_mode;
        ROS_INFO("cym_planner: switched to %s mode for the new plan.",
                 holonomic_mode_ ? "holonomic" : "normal");
    }
    double requested_task_max_vel = 0.0;
    if (canonical_nh.getParam("task_max_vel", requested_task_max_vel))
        task_max_vel_ = std::max(0.0, requested_task_max_vel);
    else
        task_max_vel_ = 0.0;
    target_index_ = 0;
    global_plan_ = plan;
    pose_adjusting_ = false;
    goal_reached_ = false;
    linear_derivative_initialized_ = false;
    lateral_derivative_initialized_ = false;
    angular_derivative_initialized_ = false;
    return true;
}

bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    if (global_plan_.empty() || !costmap_ros_ || !tf_listener_)
        return false;

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    const int check_start_index = std::max(
        0, std::min(target_index_, static_cast<int>(global_plan_.size()) - 1));
    double checked_distance = 0.0;
    double previous_x = 0.0;
    double previous_y = 0.0;
    bool have_previous_point = false;

    // This planner does not detour.  A blocked path segment causes move_base to
    // stop and attempt recovery/replanning instead of sending an unsafe command.
    for (size_t i = 0; i < global_plan_.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_costmap;
        geometry_msgs::PoseStamped plan_pose = global_plan_[i];
        plan_pose.header.stamp = ros::Time(0);
        try
        {
            tf_listener_->transformPose(costmap_frame, plan_pose, pose_costmap);
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform plan into %s: %s",
                              costmap_frame.c_str(), ex.what());
            return false;
        }

        if (static_cast<int>(i) < check_start_index)
            continue;

        if (have_previous_point)
        {
            checked_distance += std::hypot(
                pose_costmap.pose.position.x - previous_x,
                pose_costmap.pose.position.y - previous_y);
        }
        previous_x = pose_costmap.pose.position.x;
        previous_y = pose_costmap.pose.position.y;
        have_previous_point = true;

        if (checked_distance > obstacle_lookahead_distance_)
            break;

        unsigned int x = 0;
        unsigned int y = 0;
        if (costmap->worldToMap(pose_costmap.pose.position.x, pose_costmap.pose.position.y, x, y) &&
            costmap->getCost(x, y) >= obstacle_cost_threshold_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "cym_planner: blocked path segment; requesting recovery/replan");
            return false;
        }
    }

    geometry_msgs::PoseStamped pose_final;
    geometry_msgs::PoseStamped final_plan_pose = global_plan_.back();
    final_plan_pose.header.stamp = ros::Time(0);
    try
    {
        tf_listener_->transformPose(base_link_frame_, final_plan_pose, pose_final);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform final goal: %s", ex.what());
        return false;
    }

    const double final_distance = std::hypot(
        pose_final.pose.position.x, pose_final.pose.position.y);
    if (!pose_adjusting_ && final_distance < 0.05)
        pose_adjusting_ = true;

    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    const double active_max_vel_x =
        task_max_vel_ > 0.0 ? std::min(max_vel_x_, task_max_vel_) : max_vel_x_;
    const double active_max_vel_y =
        task_max_vel_ > 0.0 ? std::min(max_vel_y_, task_max_vel_) : max_vel_y_;
    if (pose_adjusting_)
    {
        const double final_yaw = tf::getYaw(pose_final.pose.orientation);
        // Translation has already reached the position tolerance.  Do not mix
        // it with final orientation alignment: the robot rotates only now.
        cmd_vel.angular.z = std::max(
            -final_yaw_max_vel_ * motion_scale,
            std::min(final_yaw * final_yaw_gain_ * motion_scale,
                     final_yaw_max_vel_ * motion_scale));
        if (std::abs(final_yaw) < final_yaw_tolerance_)
        {
            goal_reached_ = true;
            cmd_vel = geometry_msgs::Twist();
        }
        return true;
    }

    geometry_msgs::PoseStamped target_pose = pose_final;
    for (size_t i = target_index_; i < global_plan_.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_base;
        geometry_msgs::PoseStamped plan_pose = global_plan_[i];
        plan_pose.header.stamp = ros::Time(0);
        try
        {
            tf_listener_->transformPose(base_link_frame_, plan_pose, pose_base);
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform target pose: %s", ex.what());
            return false;
        }
        target_pose = pose_base;
        if (std::hypot(pose_base.pose.position.x, pose_base.pose.position.y) > 0.2)
        {
            target_index_ = static_cast<int>(i);
            break;
        }
    }

    const ros::Time control_time = ros::Time::now();
    if (!holonomic_mode_)
    {
        const double heading_error = std::atan2(
            target_pose.pose.position.y, target_pose.pose.position.x);
        double heading_error_derivative = 0.0;
        if (angular_derivative_initialized_)
        {
            const double control_period =
                (control_time - previous_heading_control_time_).toSec();
            if (control_period > 1e-3)
            {
                heading_error_derivative = std::max(
                    -4.0,
                    std::min((heading_error - previous_heading_error_) / control_period, 4.0));
            }
        }
        previous_heading_error_ = heading_error;
        previous_heading_control_time_ = control_time;
        angular_derivative_initialized_ = true;

        const double angular_control =
            heading_error * angular_gain_ + heading_error_derivative * angular_kd_;
        cmd_vel.angular.z = std::max(
            -max_vel_theta_ * motion_scale,
            std::min(angular_control * motion_scale,
                     max_vel_theta_ * motion_scale));
    }

    const double linear_error = target_pose.pose.position.x;
    double linear_error_derivative = 0.0;
    if (linear_derivative_initialized_)
    {
        const double control_period = (control_time - previous_control_time_).toSec();
        if (control_period > 1e-3)
        {
            linear_error_derivative = std::max(
                -2.0,
                std::min((linear_error - previous_linear_error_) / control_period, 2.0));
        }
    }
    previous_linear_error_ = linear_error;
    previous_control_time_ = control_time;
    linear_derivative_initialized_ = true;

    const double linear_control =
        (linear_error * linear_x_gain_ + linear_error_derivative * linear_x_kd_) * motion_scale;
    if (!holonomic_mode_)
    {
        cmd_vel.linear.x = std::max(
            0.0,
            std::min(linear_control, active_max_vel_x * motion_scale));
        return true;
    }

    cmd_vel.linear.x = std::max(
        -active_max_vel_x * motion_scale,
        std::min(linear_control, active_max_vel_x * motion_scale));

    const double lateral_error = target_pose.pose.position.y;
    double lateral_error_derivative = 0.0;
    if (lateral_derivative_initialized_)
    {
        const double control_period = (control_time - previous_lateral_control_time_).toSec();
        if (control_period > 1e-3)
        {
            lateral_error_derivative = std::max(
                -2.0,
                std::min((lateral_error - previous_lateral_error_) / control_period, 2.0));
        }
    }
    previous_lateral_error_ = lateral_error;
    previous_lateral_control_time_ = control_time;
    lateral_derivative_initialized_ = true;

    const double lateral_control =
        (lateral_error * linear_y_gain_ + lateral_error_derivative * linear_y_kd_) * motion_scale;
    cmd_vel.linear.y = std::max(
        -active_max_vel_y * motion_scale,
        std::min(lateral_control, active_max_vel_y * motion_scale));
    return true;
}

bool CymPlanner::isGoalReached()
{
    return goal_reached_;
}
}  // namespace cym_planner
