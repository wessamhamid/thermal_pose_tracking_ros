cmake_minimum_required(VERSION 3.0.2)
project(thermal_pose_tracking_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)