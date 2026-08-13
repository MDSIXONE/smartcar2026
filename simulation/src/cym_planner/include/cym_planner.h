#ifndef CYM_PLANNER_H_
#define CYM_PLANNER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace cym_planner
{

class CymPlanner : public nav_core::BaseLocalPlanner
{
public:
    CymPlanner();
    ~CymPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();

private:
    struct LaserPoint
    {
        double x;
        double y;
    };

    struct TrajectoryPose
    {
        double x;
        double y;
        double yaw;
    };

    struct CandidateTrajectory
    {
        double linear_velocity;
        double angular_velocity;
        double clearance;
        double score;
        bool valid;
        std::vector<TrajectoryPose> poses;
    };

    void carryModeCallback(const std_msgs::Bool::ConstPtr& message);
    void navigationModeCallback(const std_msgs::String::ConstPtr& message);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    bool setNavigationMode(const std::string& requested_mode);
    const char* navigationModeName() const;

    bool transformPlanPose(const geometry_msgs::PoseStamped& source,
                           const std::string& target_frame,
                           geometry_msgs::PoseStamped& result) const;
    bool selectTargetPose(geometry_msgs::PoseStamped& target_pose,
                          double lookahead_distance);
    bool computeMainLegacyCommands(geometry_msgs::Twist& cmd_vel,
                                   bool use_laser_projection);
    bool isCostmapPathBlocked(double lookahead_distance, int cost_threshold);
    bool checkLaserPathProjection(const std::vector<LaserPoint>& points,
                                  double lookahead_distance,
                                  bool& projection_blocked);
    bool copyFreshLaserPoints(std::vector<LaserPoint>& points,
                              ros::Time& scan_stamp) const;
    bool computeLaserBlockedCommand(geometry_msgs::Twist& cmd_vel);
    void computeSmoothedLaserCommand(
        const CandidateTrajectory& selected,
        const std::vector<LaserPoint>& laser_points,
        double front_clearance,
        geometry_msgs::Twist& cmd_vel);
    void rememberLaserCommand(const geometry_msgs::Twist& cmd_vel);
    void resetLaserBlockedState();
    void resetLaserBrakingState();

    CandidateTrajectory simulateTrajectory(double linear_velocity,
                                           double angular_velocity,
                                           const std::vector<LaserPoint>& points,
                                           double front_clearance) const;
    double clearanceToFootprint(double point_x, double point_y,
                                double robot_x, double robot_y,
                                double robot_yaw) const;
    double forwardClearance(const std::vector<LaserPoint>& points) const;
    void publishLaserPoints(const std::vector<LaserPoint>& points,
                            const ros::Time& stamp) const;
    void publishTrajectoryDebug(const std::vector<CandidateTrajectory>& candidates,
                                int selected_index) const;
    void publishLookaheadFootprint(const geometry_msgs::PoseStamped& lookahead_pose,
                                   const std::string& costmap_frame) const;
    void publishSafetyState(const std::string& state) const;

    bool initialized_;
    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::string base_link_frame_;
    std::string scan_topic_;
    double lookahead_distance_;
    double linear_x_gain_;
    double angular_gain_;
    double max_vel_x_;
    double max_vel_theta_;
    double final_yaw_gain_;
    double final_yaw_max_vel_;
    double final_yaw_tolerance_;
    double final_linear_x_gain_;
    double goal_position_tolerance_;
    double carry_speed_scale_;
    // Exact origin/main controller values used before pickup.  They are kept
    // separate so the post-pickup laser rollout can evolve independently.
    double main_legacy_target_distance_;
    double main_legacy_linear_x_gain_;
    double main_legacy_linear_x_kd_;
    double main_legacy_angular_gain_;
    double main_legacy_max_vel_x_;
    double main_legacy_max_vel_theta_;
    double main_legacy_final_yaw_gain_;
    double main_legacy_final_yaw_max_vel_;
    double main_legacy_final_yaw_tolerance_;
    double main_legacy_final_linear_x_gain_;
    double main_legacy_goal_position_tolerance_;
    double main_legacy_obstacle_lookahead_distance_;
    int main_legacy_obstacle_cost_threshold_;
    double main_legacy_previous_linear_error_;
    ros::Time main_legacy_previous_control_time_;
    bool main_legacy_linear_derivative_initialized_;

    double scan_timeout_;
    double scan_min_range_;
    double scan_max_range_;
    double laser_projection_step_;
    double safety_margin_;
    double braking_deceleration_;
    double minimum_moving_clearance_;
    double laser_blocked_grace_period_;
    double laser_blocked_hold_max_velocity_;
    double laser_blocked_stop_deceleration_;
    double laser_blocked_stop_angular_deceleration_;
    double laser_command_linear_rate_limit_;
    double laser_command_angular_rate_limit_;
    double reaction_time_;
    double simulation_time_;
    double simulation_step_;
    int v_samples_;
    int w_samples_;
    double path_distance_weight_;
    double heading_weight_;
    double clearance_weight_;
    double velocity_weight_;
    double angular_velocity_weight_;
    double minimum_progress_velocity_;
    double minimum_turn_velocity_;
    double footprint_min_x_;
    double footprint_max_x_;
    double footprint_min_y_;
    double footprint_max_y_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool carry_mode_;
    geometry_msgs::Twist previous_laser_command_;
    ros::Time previous_laser_command_time_;
    ros::Time laser_blocked_since_;
    bool laser_blocked_zero_published_;
    // true: origin/main path follower guarded by the direct-laser vehicle
    // projection; false: origin/main path follower guarded by its costmap.
    std::atomic<bool> laser_avoidance_enabled_;

    mutable std::mutex scan_mutex_;
    std::vector<LaserPoint> laser_points_;
    ros::Time last_scan_stamp_;
    bool have_scan_;

    ros::Subscriber carry_mode_sub_;
    ros::Subscriber navigation_mode_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher laser_points_pub_;
    ros::Publisher candidate_trajectories_pub_;
    ros::Publisher selected_trajectory_pub_;
    ros::Publisher lookahead_footprint_pub_;
    ros::Publisher safety_state_pub_;
};

}  // namespace cym_planner

#endif  // CYM_PLANNER_H_
