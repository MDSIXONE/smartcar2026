#include <ros/ros.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <sensor_msgs/Imu.h>//imu数据信息
#include <geometry_msgs/Point32.h>//geometry_msgs消息数据类型
#include "std_msgs/Int8.h"
 
double Pi = 3.14;
class Quat_to_angle
{
    public: 
     //创建需要的发布对象
     geometry_msgs::Point32 rpy_pt;

     std_msgs::Int8 imu_flag;
 
     //创建发布者
     ros::Publisher rpy_publisher;
 
     //创建订阅者，订阅IMU.orientation转换后的欧拉角信息
     ros::Subscriber rpy;
 
     //创建句柄节点
     ros::NodeHandle nh;
     
     //创建构造函数，在构造函数里给订阅者和发布者初始化
     Quat_to_angle(ros::NodeHandle nh)
     {      
       rpy_publisher = nh.advertise<std_msgs::Int8>("/pub",1); 
 
       //在调用类内部的回调函数时，callback前要加入命名空间，并且要在后面加this指针，表示调用自己内部函数
       rpy = nh.subscribe<sensor_msgs::Imu>("/imu", 1000, &Quat_to_angle::ImuCallback, this);
 
     };
     
     //创建成员回调函数，实现具体转换功能
     void ImuCallback(const sensor_msgs::ImuConstPtr& imu)  
        {   //传入数据时，使用常量指针的形式，让数据不可改变，这样比较规范

            int count = 0;

            //定义一个四元数quadf
            tf::Quaternion quat;
            
            //把msg形式的四元数转化为tf形式,得到quat的tf形式
            tf::quaternionMsgToTF(imu->orientation, quat);
 
            //定义存储r\p\y的容器
            double roll, pitch, yaw;
 
            //进行转换得到RPY欧拉角
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            //定义将要发布的欧拉角数据类型           
            rpy_pt.x = roll*180/ Pi;
            rpy_pt.y = pitch*180/ Pi;
            rpy_pt.z = yaw*180/ Pi;
 
            std::cout<<"roll="<<rpy_pt.x<<"\t pitch="<<rpy_pt.y<<"\t yaw="<<rpy_pt.z<<std::endl;
            if(rpy_pt.y<-2)
            {
                count = 1;
                imu_flag.data = count;
                rpy_publisher.publish(imu_flag);  
            }
            else if(rpy_pt.y<10&&rpy_pt.y>1)
            {
                count = 2; 
                imu_flag.data = count;
                rpy_publisher.publish(imu_flag);
            }
            ROS_INFO("-------------%d--------------",imu_flag.data);
            // rpy_publisher.publish(rpy_pt);
        }       
};
int main(int argc, char **argv)
{
    //初始化ROS节点
    ros::init(argc, argv, "quat_to_angle");
    //创建句柄节点
    ros::NodeHandle nh;
    //创建实例化对象q
    Quat_to_angle q(nh);
    // q.rpy_publisher = nh.advertise<geometry_msgs::Point32>("/pub",1000); 
    // //如果是在类外进行订阅，则需要将this，换为&q(实例化的对象)，表示调用q对象上的回调函数
    // q.rpy = nh.subscribe<sensor_msgs::Imu>("/imu_data", 1000, &Quat_to_angle::ImuCallback, &q);
 
    // 检查传入的四元数直到按下crtl+c
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}