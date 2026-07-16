#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

// 发布器全局变量
ros::Publisher cmd_vel_pub;
ros::Publisher mission_complete_pub;
ros::Publisher mission_second_pub;

bool mission_started = false;
bool distance = false;
int mission_stage = 0; // 0: 未开始, 1: 到 0.6 米, 2: 对准板子, 3: 到 0.4 米

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> find_board_edges(const std::vector<float>& ranges, int center_index, int max_points, float threshold)
{
    std::vector<float> left_valid_ranges;
    std::vector<float> right_valid_ranges;
    std::vector<float> mid_valid_ranges;
    // 从中心向左扫描
    for (int i = center_index; i >= 0 && left_valid_ranges.size() < max_points; --i)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > 0.05 && ranges[i] < 5.0)
        {
            if (left_valid_ranges.empty() || std::abs(ranges[i] - ranges[center_index]) < threshold)
            {
                left_valid_ranges.push_back(ranges[i]);
            }
            else
            {
                break;
            }
        }
    }

    // 从中心向右扫描
    for (int i = center_index + 1; i < ranges.size() && right_valid_ranges.size() < max_points; ++i)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > 0.05 && ranges[i] < 5.0)
        {
            if (right_valid_ranges.empty() || std::abs(ranges[i] - ranges[center_index]) < threshold)
            {
                right_valid_ranges.push_back(ranges[i]);
            }
            else
            {
                break;
            }
        }
    }
    // 中间扫描
    int start_mid_index = center_index - max_points / 5;
    int end_mid_index = center_index + max_points / 5;
    for (int i = start_mid_index; i <= end_mid_index && mid_valid_ranges.size() < max_points; ++i)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > 0.05 && ranges[i] < 5.0)
        {
            mid_valid_ranges.push_back(ranges[i]);
        }
    }

    return std::make_tuple(left_valid_ranges, right_valid_ranges, mid_valid_ranges);
    // return std::make_pair(left_valid_ranges, right_valid_ranges);
}
// 计算平均值
float calculate_average(const std::vector<float>& valid_ranges, int num_points)
{
    if (valid_ranges.empty())
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // 取最后 num_points 个数据点进行计算平均值
    int start_index = valid_ranges.size() - num_points;
    start_index = std::max(0, start_index);  // Ensure start_index is not negative

    float sum = 0.0f;
    for (int i = start_index; i < valid_ranges.size(); ++i)
    {
        sum += valid_ranges[i];
    }

    return sum / (valid_ranges.size() - start_index);
}

float calculate_average_mid(const std::vector<float>& valid_ranges)
{
    if (valid_ranges.empty())
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    return std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f) / valid_ranges.size();
}

// // 计算多点平均值函数，排除突变值
// float filtered_average(const std::vector<float>& ranges, int start, int end)
// {
//     std::vector<float> valid_ranges;
//     for (int i = start; i <= end; ++i)
//     {
//         if (std::isfinite(ranges[i]) && ranges[i] > 0.05 && ranges[i] < 5.0) // 过滤无效和过大的数据
//         {
//             valid_ranges.push_back(ranges[i]);
//         }
//     }

//     // 排除突变值
//     std::sort(valid_ranges.begin(), valid_ranges.end());
//     int size = valid_ranges.size();
//     if (size > 4)
//     {
//         valid_ranges = std::vector<float>(valid_ranges.begin() + size / 4, valid_ranges.end() - size / 4); // 去掉前后各25%的数据
//     }

//     if (valid_ranges.empty())
//     {
//         return std::numeric_limits<float>::quiet_NaN();
//     }
//     return std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0) / valid_ranges.size();
// }

// 激光雷达回调函数
void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!mission_started)
    {
        return; // 如果任务没有开始，不执行回调函数
    }
    std::vector<float> ranges = msg->ranges;

    int center_index = 175;  //175
    int max_points = 10;
    float threshold = 0.1;
    int num_points_for_average = 3;  // Number of points to use for averaging

    auto [left_valid_ranges, right_valid_ranges, mid_valid_ranges] = find_board_edges(ranges, center_index, max_points, threshold);

    // // 输出左侧有效范围
    // std::cout << "Left valid ranges:" << std::endl;
    // for (size_t i = 0; i < left_valid_ranges.size(); ++i)
    // {
    //     std::cout << "Left[" << i << "]: " << left_valid_ranges[i] << std::endl;
    // }

    // // 输出右侧有效范围
    // std::cout << "Right valid ranges:" << std::endl;
    // for (size_t i = 0; i < right_valid_ranges.size(); ++i)
    // {
    //     std::cout << "Right[" << i << "]: " << right_valid_ranges[i] << std::endl;
    // }


    // Calculate average of last num_points_for_average points for left and right ranges
    float left_distance = calculate_average(left_valid_ranges, num_points_for_average);
    float right_distance = calculate_average(right_valid_ranges, num_points_for_average);
    float mid_distance = calculate_average_mid(mid_valid_ranges);
    // Output left and right averages
    std::cout << "left_distance: " << left_distance << std::endl;
    std::cout << "right_distance: " << right_distance << std::endl;
    std::cout << " mid_distance: " <<  mid_distance << std::endl;
    // // 假设激光雷达在板子前面数据的索引范围
    // int start_index = 165;
    // int end_index = 185;

    // float left_distance = filtered_average(msg->ranges, start_index, start_index + 5);  // 左侧多点平均值
    // float right_distance = filtered_average(msg->ranges, end_index - 5, end_index);    // 右侧多点平均值
    // float mid_distance = filtered_average(msg->ranges, start_index + 5, end_index - 5); // 中间多点平均值

    // ROS_INFO("左侧距离: %f 米, 右侧距离: %f 米, 中间距离: %f 米", left_distance, right_distance, mid_distance);

    // 检查数据有效性
    // if (!std::isfinite(left_distance) || !std::isfinite(right_distance) || !std::isfinite(mid_distance))
    // {
    //     ROS_WARN("激光雷达数据无效");
    //     return;
    // }

    geometry_msgs::Twist twist;

    if (mission_stage == 1)
    {   
        if (mid_distance > 1.00 || distance  )
        {   
             ROS_INFO("到达1m处");
             distance = true; 
        
             if (mid_distance <0.9)
             {

                twist.linear.x = 0.0;  // 停止
                twist.angular.z = 0.0; // 停止转向
                cmd_vel_pub.publish(twist);
                mission_started = false;  // 停止任务
                distance = false;
                ROS_INFO("到达0.9m处");

                // 发布任务完成信号
                std_msgs::Int32 second_msg;
                second_msg.data = mission_stage;
                mission_second_pub.publish(second_msg);

                // 发布任务完成信号
                std_msgs::Int32 complete_msg;
                complete_msg.data = 1;
                mission_complete_pub.publish(complete_msg);
               
            }
            else 
            {
                twist.linear.x = 0.3;  // 前进速度
                twist.angular.z = 0.0;
                cmd_vel_pub.publish(twist);
            }
           
          
        }
        else {
            ROS_INFO(" mission_stage = 2");
            mission_stage = 2;
        }
        
        
    }
    if (mission_stage == 2)
    {   
        
        // 判断是否到达60cm处
        if (mid_distance > 0 && mid_distance < 0.50)
        {
            twist.linear.x = 0.0;  // 停止
            cmd_vel_pub.publish(twist);
            mission_stage = 3;     // 进入对准阶段
            ROS_INFO("到达50cm处");
        }
        else
        {
            twist.linear.x = 0.3;  // 前进速度
            twist.angular.z = 0.0;
            cmd_vel_pub.publish(twist);
        }
    }
    else if (mission_stage == 3)
    {
        // 计算板子的倾斜度
        float angle_to_board = atan2(right_distance - left_distance, 0.5); // 假设板子宽度为50cm

        // 调整小车的方向以对准板子
        twist.linear.y = angle_to_board * 0.7; // 调整线速度的y分量，可以根据实际情况调整比例系数0.6
        twist.angular.z = -angle_to_board; // 调整角速度

        if (fabs(angle_to_board) < 0.04)
        {
            mission_stage = 4;  // 进入移动到40cm处阶段s
            ROS_INFO("板子对准完成");
        }
        cmd_vel_pub.publish(twist);
    }
    else if (mission_stage == 4)
    {
        // 判断是否到达40cm处
        if (mid_distance > 0 && mid_distance < 0.45)
        {
            twist.linear.x = 0.0;  // 停止
            twist.angular.z = 0.0; // 停止转向
            cmd_vel_pub.publish(twist);
            mission_started = false;  // 停止任务
            ROS_INFO("到达45cm处");

            // 发布任务完成信号
            std_msgs::Int32 complete_msg;
            complete_msg.data = 1;
            mission_complete_pub.publish(complete_msg);
        }
        else
        {
            twist.linear.x = 0.3;  // 前进速度
            twist.angular.z = 0.0;
            cmd_vel_pub.publish(twist);
        }
    }
}

// 开始任务的回调函数
void StartMissionCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 1)
    {
        mission_started = true;
        mission_stage = 1;
        ROS_INFO("任务开始");
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");                   // 设置中文编码
    ros::init(argc, argv, "lidar_node");     // 初始化节点

    ros::NodeHandle n;                       // 创建节点句柄

    // 订阅激光雷达话题
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, LidarCallback);

    // 初始化速度命令发布器
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 订阅开始任务的话题
    ros::Subscriber start_mission_sub = n.subscribe("/start_mission", 10, StartMissionCallback);

    // 初始化任务完成发布器
    mission_complete_pub = n.advertise<std_msgs::Int32>("/mission_complete", 10);

    // 初始化任务完成发布器
    mission_second_pub = n.advertise<std_msgs::Int32>("/mission_second", 10);

    ros::spin();
    return 0;
}
