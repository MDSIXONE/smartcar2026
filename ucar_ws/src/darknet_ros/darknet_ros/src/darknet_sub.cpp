#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "std_msgs/Int8.h"
#include "string.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>
using namespace std;

void fileEmpty(const string fileName)
{
    fstream file(fileName, ios::out);
}
int Write_Class(string Class,float Prob,int a,int b) {
	fstream f;
	f.open("/home/ucar/ucar_ws/src/txt_total/" + to_string(a) + "_" + to_string(b) + ".txt", ios::out | ios::app);
	f << Class << ':' << Prob << endl;
	f.close();
	return 0;
}

// int Read_Class(int a) {
// 	int count_1 = 0;
// 	fstream f;
// 	f.open("/home/ucar/ucar_ws/src/txt_total/" + to_string(a) + ".txt", ios::in);
// 	string s;
// 	while (f >> s)
// 	{
// 		// cout << s << endl; //显示读取内容 
// 		if (s == "watermelon")
// 			count_1++;
// 	}
// 	// cout << count_1<<endl;
// 	f.close();
// 	return count_1;
// }
int txt_name = 0;
void darknet_yolo_callback(const std_msgs::Int8::ConstPtr& msg)
{
	txt_name = msg->data;
} 
void personTrack(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	// double xmin = msg->bounding_boxes[0].xmin;
	// double ymin = msg->bounding_boxes[0].ymin;
	// double xmax = msg->bounding_boxes[0].xmax;
	// double ymax = msg->bounding_boxes[0].ymax;
    std::string class_name = msg->bounding_boxes[0].Class;
	float prob_get = msg->bounding_boxes[0].probability;
    // ROS_INFO("%s------------%f",class_name.c_str(),prob_get);
	if(txt_name)
	{
	Write_Class(class_name,prob_get,txt_name/10,txt_name%10);
	}
    // Write_Class(class_name,2);
    // ROS_INFO("-------------%d------------",Read_Class(2));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet_classes");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",1,personTrack);
    ros::Subscriber yolo_sub = nh.subscribe("/darknet_yolo",1,darknet_yolo_callback);
	// fstream file("/home/ucar/ucar_ws/src/txt_total/1_1.txt", ios::out);
	fileEmpty("/home/ucar/ucar_ws/src/txt_total/1_1.txt");
	fileEmpty("/home/ucar/ucar_ws/src/txt_total/1_2.txt");
	fileEmpty("/home/ucar/ucar_ws/src/txt_total/1_3.txt");
	fileEmpty("/home/ucar/ucar_ws/src/txt_total/1_4.txt");
    ros::spin();
    return 0;
}