
#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>


int main(int argc, char** argv)
{

	ros::init(argc, argv, "kill");    

	ros::NodeHandle nha;
	
	system("rosnode kill /follower");
	system("rosnode kill /usb_cam");
	system("rosnode kill /visual_tracker");    
	system("rosnode kill /camera/driver");
	system("rosnode kill /camera/camera_nodelet_manager");
	system("rosnode kill /camera/depth_metric");
	system("rosnode kill /camera/depth_metric_rect");
	system("rosnode kill /camera/depth_points");
	system("rosnode kill /camera/depth_rectify_depth");
	system("rosnode kill /camera/depth_registered_hw_metric_rect");
	system("rosnode kill /camera/depth_registered_metric");
	system("rosnode kill /camera/depth_registered_rectify_depth");
	system("rosnode kill /camera/points_xyzrgb_hw_registered");
	system("rosnode kill /camera/rgb_rectify_color");
	system("rosnode kill /camera_base_link");
	system("rosnode kill /camera_base_link1");
	system("rosnode kill /camera_base_link2");
	system("rosnode kill /camera_base_link3");


	return 0;

}

