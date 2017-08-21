//
// Created by andrew on 8/19/17.
//

#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    std::cout << "Starting Mono VO node with Cam info...  " << std::endl;
    ros::init(argc, argv, "mono_vo");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "usb_cam/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "usb_cam/camera_info", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
//  image_transport::ImageTransport it(nh);
//  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}