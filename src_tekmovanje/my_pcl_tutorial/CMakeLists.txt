cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_tutorial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
	tf
	cv_bridge
	rospy
	std_msgs
	image_transport
	genmsg geometry_msgs visualization_msgs
)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs visualization_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

