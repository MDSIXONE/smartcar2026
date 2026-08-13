#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <std_msgs/String.h>
#include <sstream>
#include <iomanip>

#include <std_msgs/Int32.h>
// 聚类结构体
struct Cluster {
    std::vector<int> indices;
};
ros::Publisher pub;
float dist=1.0;
int skip=1;
int counter=0;

int on=0;
void cb(const std_msgs::Int32::ConstPtr& msg) {
on=msg->data;
}


void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
if(on==1)return;
     counter++;
     if(counter<skip){
     return;
     }else{
     counter=0;
     }
     
     float MAX_DIST = dist;  // 1米距离阈值
     float ANGLE_OFFSET = 0.0;  // 角度偏移校准

    // 提取扫描数据
    const std::vector<float>& ranges = scan->ranges;
    const float angle_min = scan->angle_min;
    const float angle_increment = scan->angle_increment;
    const int num_points = ranges.size();

    // 步骤1: 筛选1米内的有效点
    std::vector<int> valid_indices;
    for (int i = 0; i < num_points; ++i) {
        if (ranges[i] > 0 && ranges[i] <= MAX_DIST) {
            valid_indices.push_back(i);
        }
    }

    if (valid_indices.empty()) {
        ROS_INFO("No board detected within 1 meter");
        std_msgs::String msg;
        
        
        msg.data = "x";
        
        // 发布消息
        pub.publish(msg);
        return;
    }

    // 步骤2: 聚类相邻索引
    std::vector<Cluster> clusters;
    Cluster current_cluster;
    current_cluster.indices.push_back(valid_indices[0]);

    for (size_t i = 1; i < valid_indices.size(); ++i) {
        if (valid_indices[i] - valid_indices[i-1] == 1) {  // 连续点
            current_cluster.indices.push_back(valid_indices[i]);
        } else {
            clusters.push_back(current_cluster);
            current_cluster.indices.clear();
            current_cluster.indices.push_back(valid_indices[i]);
        }
    }
    clusters.push_back(current_cluster);

    // 步骤3: 处理环形边缘（首尾聚类合并）
    if (clusters.size() > 1) {
        Cluster& first_cluster = clusters.front();
        Cluster& last_cluster = clusters.back();
        //printf("%d %d %d",first_cluster.indices.front(),last_cluster.indices.back(),num_points-1);
        // 检查首尾是否在边缘连续
        if (first_cluster.indices.front() <= 1 && 
            last_cluster.indices.back() == num_points - 1) {
            
            // 合并首尾聚类
            last_cluster.indices.insert(
                last_cluster.indices.end(), 
                first_cluster.indices.begin(), 
                first_cluster.indices.end()
            );
            
            // 移除首聚类
            clusters.erase(clusters.begin());
        }
    }

    // 步骤4: 找出最大聚类
    auto largest_cluster = std::max_element(clusters.begin(), clusters.end(),
        [](const Cluster& a, const Cluster& b) {
            return a.indices.size() < b.indices.size();
        });

    // 步骤5: 计算中心点距离和角度
    double total_dist = 0.0;
    double x_sum = 0.0, y_sum = 0.0;
    
    float a0,a1;
    float s0,s1;
    int idx0=-1000;
    int idx1;
    for (int idx : largest_cluster->indices) {
        idx1=idx;
        if(idx0==-1000)idx0=idx;
        float dist = ranges[idx];
        total_dist += dist;
        
        float angle = angle_min + idx * angle_increment;
        x_sum += dist * cos(angle);
        y_sum += dist * sin(angle);
    }
    
    a0=angle_min + idx0 * angle_increment;
    a1=angle_min + idx1 * angle_increment;
    s0=ranges[idx0];
    s1=ranges[idx1];
    
    
    double avg_dist = total_dist / largest_cluster->indices.size();
    double avg_angle = atan2(y_sum, x_sum) + ANGLE_OFFSET;
    
    
    std_msgs::String msg;
        
         std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);  // 设置固定小数点和精度
    oss << avg_dist << "|" << (avg_angle * 180.0 / M_PI)<<
    "|" << s0<<
    "|" << (a0 * 180.0 / M_PI)<<
    "|" << s1<<
    "|" << (a1 * 180.0 / M_PI)
    ;
    
    
        msg.data = oss.str();
        
        // 发布消息
        pub.publish(msg);
    
    
    ROS_INFO("Board detected: Distance = %.2fm, Angle = %.2fdeg", 
             avg_dist, avg_angle * 180.0 / M_PI);
    ROS_INFO("Board left: Distance = %.2fm, Angle = %.2fdeg", 
             s0, (a0 * 180.0 / M_PI));
    ROS_INFO("Board right: Distance = %.2fm, Angle = %.2fdeg", 
             s1, (a1 * 180.0 / M_PI));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "board_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); 
    private_nh.getParam("dist", dist); 
    private_nh.getParam("skip", skip); 
    
    
    printf("%f",dist);
    ros::Subscriber sub2 = nh.subscribe<std_msgs::Int32>(
        "/bd_start", 10, cb);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>(
        "/scan", 10, laserscanCallback);  // 替换为实际话题名
    pub = nh.advertise<std_msgs::String>("/bd_result", 10);
    ros::spin();
    return 0;
}