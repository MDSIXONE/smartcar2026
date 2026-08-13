#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
cv::Mat imgCallback;
image_transport::Publisher image_pub;
static void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
      // cout<<"FLIR time:"<<msg->header.stamp<<endl;
      // return;
      // msg = msg[:,:,::-1]
      image_pub.publish(msg);
      // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      // imgCallback = cv_ptr->image;
      // cv::imshow("imgCallback",imgCallback);
      // cv::waitKey(1);
      // cout<<"cv_ptr: "<<cv_ptr->image.cols<<" h: "<<cv_ptr->image.rows<<endl;
    }
    catch (cv_bridge::Exception& e)
    {
      //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      //ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->format.c_str());
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CompressedImage");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  std::string image_topic = "/usb_cam/image_raw";
  image_sub = it.subscribe(image_topic,1,ImageCallback);
  image_pub = it.advertise("/usb_cam_rgb",1);
  ros::spin();
  return 0;
}

