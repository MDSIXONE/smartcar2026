#include "cym_planner/inflation_recovery_schedule.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace
{

const double kAppliedRadiusTolerance = 1e-6;
const double kServiceWaitSeconds = 5.0;

bool getInflationRadius(
    ros::ServiceClient& client,
    const std::string& service_name,
    double& current_radius)
{
    if(!client.waitForExistence(ros::Duration(kServiceWaitSeconds)))
    {
        ROS_ERROR(
            "cym_planner: inflation recovery service %s is unavailable",
            service_name.c_str());
        return false;
    }

    dynamic_reconfigure::Reconfigure request;
    if(!client.call(request))
    {
        ROS_ERROR(
            "cym_planner: failed to read inflation radius via %s",
            service_name.c_str());
        return false;
    }

    for(std::size_t index = 0;
        index < request.response.config.doubles.size();
        ++index)
    {
        const dynamic_reconfigure::DoubleParameter& response_parameter =
            request.response.config.doubles[index];
        if(response_parameter.name == "inflation_radius")
        {
            current_radius = response_parameter.value;
            if(std::isfinite(current_radius))
                return true;
            break;
        }
    }

    ROS_ERROR(
        "cym_planner: inflation radius returned by %s is invalid",
        service_name.c_str());
    return false;
}

bool setInflationRadius(
    ros::ServiceClient& client,
    const std::string& service_name,
    double target_radius,
    double& applied_radius)
{
    if(!client.waitForExistence(ros::Duration(kServiceWaitSeconds)))
    {
        ROS_ERROR(
            "cym_planner: inflation recovery service %s is unavailable",
            service_name.c_str());
        return false;
    }

    dynamic_reconfigure::Reconfigure request;
    dynamic_reconfigure::DoubleParameter parameter;
    parameter.name = "inflation_radius";
    parameter.value = target_radius;
    request.request.config.doubles.push_back(parameter);
    if(!client.call(request))
    {
        ROS_ERROR(
            "cym_planner: failed to set inflation radius %.3f via %s",
            target_radius, service_name.c_str());
        return false;
    }

    bool found_radius = false;
    for(std::size_t index = 0;
        index < request.response.config.doubles.size();
        ++index)
    {
        const dynamic_reconfigure::DoubleParameter& response_parameter =
            request.response.config.doubles[index];
        if(response_parameter.name == "inflation_radius")
        {
            applied_radius = response_parameter.value;
            found_radius = true;
            break;
        }
    }
    if(!found_radius ||
       !std::isfinite(applied_radius) ||
       std::abs(applied_radius - target_radius) > kAppliedRadiusTolerance)
    {
        ROS_ERROR(
            "cym_planner: inflation radius %.3f did not apply via %s; "
            "returned %.9f",
            target_radius, service_name.c_str(), applied_radius);
        return false;
    }
    return true;
}

template<typename T>
T requiredParam(
    const ros::NodeHandle& node_handle,
    const std::string& key,
    const std::string& behavior_name)
{
    T value;
    if(!node_handle.getParam(key, value))
    {
        throw std::runtime_error(
            "cym_planner: missing parameter ~" + key +
            " for recovery behavior " + behavior_name);
    }
    return value;
}

}  // namespace

namespace cym_planner
{

class InflationRecovery : public nav_core::RecoveryBehavior
{
public:
    InflationRecovery()
        : initialized_(false),
          stage_(0),
          local_reduction_step_(0.0),
          global_reduction_step_(0.0),
          minimum_radius_(0.0)
    {
    }

    void initialize(
        std::string name,
        tf2_ros::Buffer* /* tf */,
        costmap_2d::Costmap2DROS* /* global_costmap */,
        costmap_2d::Costmap2DROS* /* local_costmap */) override
    {
        if(initialized_)
            return;

        ros::NodeHandle behavior_nh("~/" + name);
        ros::NodeHandle config_nh("~");
        stage_ = requiredParam<int>(behavior_nh, "stage", name);
        const double local_reduction_step = requiredParam<double>(
            config_nh, "inflation_recovery/local_reduction_step", name);
        const double global_reduction_step = requiredParam<double>(
            config_nh, "inflation_recovery/global_reduction_step", name);
        const double minimum_radius = requiredParam<double>(
            config_nh, "inflation_recovery/minimum_radius", name);
        local_service_name_ = requiredParam<std::string>(
            config_nh, "inflation_recovery/local_service", name);
        global_service_name_ = requiredParam<std::string>(
            config_nh, "inflation_recovery/global_service", name);

        if(stage_ <= 0 ||
           !std::isfinite(local_reduction_step) ||
           !std::isfinite(global_reduction_step) ||
           !std::isfinite(minimum_radius) ||
           local_reduction_step <= 0.0 ||
           global_reduction_step <= 0.0 ||
           minimum_radius < 0.0)
        {
            throw std::runtime_error(
                "cym_planner: invalid inflation recovery schedule for " +
                name);
        }
        local_reduction_step_ = local_reduction_step;
        global_reduction_step_ = global_reduction_step;
        minimum_radius_ = minimum_radius;

        ros::NodeHandle service_nh;
        local_client_ = service_nh.serviceClient<
            dynamic_reconfigure::Reconfigure>(local_service_name_);
        global_client_ = service_nh.serviceClient<
            dynamic_reconfigure::Reconfigure>(global_service_name_);
        initialized_ = true;
    }

    void runBehavior() override
    {
        if(!initialized_)
        {
            throw std::runtime_error(
                "cym_planner: inflation recovery ran before initialize()");
        }

        double local_current_radius = 0.0;
        if(!getInflationRadius(
               local_client_, local_service_name_, local_current_radius))
        {
            throw std::runtime_error(
                "cym_planner: failed to read local inflation radius");
        }

        double global_current_radius = 0.0;
        if(!getInflationRadius(
               global_client_, global_service_name_, global_current_radius))
        {
            throw std::runtime_error(
                "cym_planner: failed to read global inflation radius");
        }

        const double local_target_radius = nextInflationRecoveryRadius(
            local_current_radius, local_reduction_step_, minimum_radius_);
        const double global_target_radius = nextInflationRecoveryRadius(
            global_current_radius, global_reduction_step_, minimum_radius_);
        if(!inflationRadiusIsValid(local_target_radius, minimum_radius_) ||
           !inflationRadiusIsValid(global_target_radius, minimum_radius_))
        {
            throw std::runtime_error(
                "cym_planner: invalid target radius for recovery behavior");
        }

        double local_applied_radius = 0.0;
        if(!setInflationRadius(
               local_client_, local_service_name_, local_target_radius,
               local_applied_radius))
        {
            throw std::runtime_error(
                "cym_planner: local inflation recovery stage failed");
        }

        double global_applied_radius = 0.0;
        if(!setInflationRadius(
               global_client_, global_service_name_, global_target_radius,
               global_applied_radius))
        {
            throw std::runtime_error(
                "cym_planner: global inflation recovery stage failed");
        }

        ROS_WARN(
            "cym_planner: recovery stage %d lowered local by %.3f m and "
            "global by %.6f m (local=%.3f, global=%.3f); replanning",
            stage_, local_reduction_step_, global_reduction_step_,
            local_applied_radius, global_applied_radius);
    }

private:
    bool initialized_;
    int stage_;
    double local_reduction_step_;
    double global_reduction_step_;
    double minimum_radius_;
    std::string local_service_name_;
    std::string global_service_name_;
    ros::ServiceClient local_client_;
    ros::ServiceClient global_client_;
};

}  // namespace cym_planner

PLUGINLIB_EXPORT_CLASS(cym_planner::InflationRecovery, nav_core::RecoveryBehavior)
