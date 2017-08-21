# ros_mono_vo
An OpenCV based Implementation of Monocular Visual Odometry for ROS. Inspired by Avi Singh: https://github.com/avisingh599/mono-vo

The repo will subscribe to an image topic as well as a camera info topic to pull a calibrated camera's intrinsics and distortions for this algorithm to work.

It currently uses the FAST algorithm for feature detection and tracking. 
