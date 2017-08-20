cmake_minimum_required(VERSION 2.8.3)
project(ros_mono_vo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mono_vo_node src/mono_vo.cpp)
target_link_libraries(mono_vo_node ${catkin_LIBRARIES})


