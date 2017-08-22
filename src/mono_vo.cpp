//
// Created by andrew on 8/19/17.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "../include/ros_mono_vo/vo_features.h" //TODO Don't make relative import.

//Algorithm Outline:
//Capture images: ItIt, It+1It+1,
//Undistort the above images.
//Use FAST algorithm to detect features in ItIt, and track those features to It+1It+1. A new detection is triggered if the number of features drop below a certain threshold.
//Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
//Estimate R,tR,t from the essential matrix that was computed in the previous step.
//Take scale information from some external source (like a speedometer), and concatenate the translation vectors, and rotation matrices.



#define MIN_NUM_FEAT 50 // TODO: make a yaml param

cv::Mat init_1_c, init_2_c; //These are the two color images we will use// .
cv::Mat init_1, init_2; //These are the BW images that will be converted from the color images.
cv::Mat prevImage, currImage;
cv::Mat prevImage_c, currImage_c;
cv::Mat E, R, t, mask, R_f, t_f; //variables for the function.
//cv::Mat prevPts, currPts;
std::vector<cv::Point2f> points1, points2, prevFeatures, currFeatures; //Vectors to store the coordinates of feature points.
std::vector<uchar> status;
double focal_length;
double scale = 2.00;

bool init= false;


void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  try
  {
      cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
      // Init logic to ensure two images are captured for initial features
      if (!init)
      {
          init_2_c = cv_bridge::toCvShare(image, "bgr8")->image;
          if (!init_1_c.empty() && !init_2_c.empty()) {
              std::cout << "Initializing..." << std::endl;

              //Convert to grayscale.
              cv::cvtColor(init_1_c, init_1, COLOR_BGR2GRAY);
              cv::cvtColor(init_2_c, init_2, COLOR_BGR2GRAY);

              //feature detection and tracking.
              featureDetection(init_1, points1); //detect features in Img1
              featureTracking(init_1, init_2, points1, points2, status); //track those features to init_2

              //focal_length = cam_info->K[0];
              focal_length = 1200.0;
              std::cout << "Focal Length: " << focal_length << std::endl;
              cv::Point2d ppoint(cam_info->K[2], cam_info->K[5]); //principle point
              std::cout << "Finding essential matrix..." << std::endl;
              E = cv::findEssentialMat(points1, points2, focal_length, ppoint, RANSAC, 0.999, 1.0, mask);
              std::cout << "E = "<< std::endl << " "  << E << std::endl << std::endl;
              cv::recoverPose(E, points1, points2, R, t, focal_length, ppoint, mask);

              /*TODO Stay in init loop until R and t are "different" enough? To ensure good data?
                 as it stands, there are just to consecutive images taken.
                 */

              prevImage = init_2;
              prevFeatures = points1;

              std::cout << "R = "<< std::endl << " "  << R << std::endl << std::endl;
              std::cout << "t = "<< std::endl << " "  << t << std::endl << std::endl;

              R_f = R.clone();
              t_f = t.clone();

              init = true;
          }

          init_1_c = init_2_c;
      }

      if(init) //Main portion of callback right now.
      {
          std::cout << "Visual Odom Publishing..." << std::endl;
      }
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