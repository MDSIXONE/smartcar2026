#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include "costmap_2d/InflationPluginConfig.h"

int main(int argc, char **argv) {
    // ros::init(argc, argv, "dynamic_reconfigure_client");
    // dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client("/move_base/local_costmap/inflation_layer/");
    // costmap_2d::InflationPluginConfig config;

    // while (ros::ok()) {
    //     config.inflation_radius = 0.1;
    //     client.setConfiguration(config);
    // }
    return 0;
}