#ifndef CYM_PLANNER_H_
#define CYM_PLANNER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "cym_planner/escape_recovery.h"
#include "cym_planner/planner_tuning.h"

#include <string>
#include <vector>

namespace cv
{
class Mat;
}

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
    struct FootprintBlockage
    {
        FootprintBlockage()
            : blocked(false),
              recoverable(false),
              current_footprint_blocked(false),
              hold(false),
              contact_count(0),
              contact_world_x(0.0),
              contact_world_y(0.0)
        {
        }

        bool blocked;
        bool recoverable;
        bool current_footprint_blocked;
        bool hold;
        unsigned int contact_count;
        double contact_world_x;
        double contact_world_y;
    };

    void carryModeCallback(const std_msgs::Bool::ConstPtr& message);
    void navigationModeCallback(const std_msgs::String::ConstPtr& message);
    bool transformPlanPose(const geometry_msgs::PoseStamped& source,
                           const std::string& target_frame,
                           geometry_msgs::PoseStamped& result) const;
    bool selectTargetPose(geometry_msgs::PoseStamped& target_pose);
    const std::vector<geometry_msgs::PoseStamped>& trackingPlan() const;
    void clearElasticPlan();
    void resetElasticSearch();
    bool elasticPlanMatchesNewGlobal(
        const std::vector<geometry_msgs::PoseStamped>& plan) const;
    bool elasticSearchPlanMatchesNewGlobal(
        const std::vector<geometry_msgs::PoseStamped>& plan) const;
    bool plansHaveSameGeometry(
        const std::vector<geometry_msgs::PoseStamped>& first,
        const std::vector<geometry_msgs::PoseStamped>& second) const;
    bool tryActivateElasticPlan(
        const std::vector<geometry_msgs::PoseStamped>& costmap_plan,
        int start_index,
        const geometry_msgs::PoseStamped& current_pose,
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image);
    bool elasticCandidateIsSafe(
        const std::vector<geometry_msgs::PoseStamped>& candidate,
        int start_index,
        int end_index,
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image);
    FootprintBlockage checkPathBlocked(
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image);
    FootprintBlockage inspectFootprint(
        const geometry_msgs::PoseStamped& pose_costmap,
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image,
        bool report_contact);
    bool computeEscapeCommand(
        const FootprintBlockage& path_blockage,
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image,
        geometry_msgs::Twist& cmd_vel);
    bool currentCostmapPose(geometry_msgs::PoseStamped& pose) const;
    bool commandSweepIsSafe(
        const geometry_msgs::Twist& command,
        const costmap_2d::Costmap2D& local_costmap,
        cv::Mat& map_image);
    bool escapePreviewIsSafe(
        const geometry_msgs::PoseStamped& current_pose,
        const costmap_2d::Costmap2D& local_costmap,
        double direction_world_x,
        double direction_world_y,
        double distance,
        cv::Mat& map_image);
    void resetEscapeRecovery();
    void publishDebugMap(const cv::Mat& map_image,
                         const std::string& frame_id) const;
    void publishDebugPlan() const;
    void resetControllerState();
    const PlannerTuning& activeTuning() const;

    bool initialized_;
    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::string base_link_frame_;
    std::string odom_frame_;
    PlannerTuning point_tuning_;
    PlannerTuning body_projection_tuning_;
    bool debug_images_enabled_;
    bool escape_enabled_;
    double escape_blocked_timeout_;
    double escape_replan_wait_;
    double escape_step_distance_;
    double escape_projection_step_;
    double escape_speed_;
    double escape_max_total_distance_;
    double escape_heading_tolerance_;
    int escape_max_attempts_;
    bool elastic_enabled_;
    double elastic_lookahead_distance_;
    double elastic_lateral_step_;
    double elastic_max_lateral_offset_;
    double elastic_validation_step_;
    double elastic_validation_yaw_step_;
    double elastic_max_vel_x_;
    double elastic_max_vel_theta_;
    double elastic_search_timeout_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool carry_mode_;
    bool body_projection_enabled_;
    bool elastic_active_;
    int elastic_end_plan_index_;
    int elastic_last_side_;
    ros::Time elastic_blocked_since_;
    std::vector<geometry_msgs::PoseStamped> elastic_search_reference_plan_;
    std::vector<geometry_msgs::PoseStamped> elastic_plan_;
    ros::Time escape_blocked_since_;
    ros::Time escape_wait_until_;
    ros::Time escape_motion_started_;
    bool escape_active_;
    int escape_attempts_;
    double escape_total_distance_;
    double escape_start_world_x_;
    double escape_start_world_y_;
    double escape_start_world_yaw_;
    double escape_direction_base_x_;
    double escape_direction_base_y_;
    double escape_direction_world_x_;
    double escape_direction_world_y_;

    double previous_linear_error_;
    ros::Time previous_linear_control_time_;
    bool linear_derivative_initialized_;
    double previous_heading_error_;
    ros::Time previous_heading_control_time_;
    bool angular_derivative_initialized_;

    ros::Subscriber carry_mode_sub_;
    ros::Subscriber navigation_mode_sub_;
    ros::Publisher debug_map_pub_;
    ros::Publisher debug_plan_pub_;
};

}  // namespace cym_planner

#endif  // CYM_PLANNER_H_
