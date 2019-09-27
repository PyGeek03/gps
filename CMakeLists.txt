cmake_minimum_required(VERSION 2.8.3)
project(gps)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
	FILES
	GPSData.msg
)
generate_messages(
	DEPENDENCIES
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
CATKIN_DEPENDS
	message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
