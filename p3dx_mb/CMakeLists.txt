cmake_minimum_required(VERSION 2.8.3)
project(p3dx_mb)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf )
catkin_package(
	
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime tf
)

include_directories(${catkin_INCLUDE_DIRS})
