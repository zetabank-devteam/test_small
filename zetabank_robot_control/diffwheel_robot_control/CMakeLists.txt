cmake_minimum_required(VERSION 2.8.3)
project(diffwheel_robot_control)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rospy
	serial
	std_msgs
	tf
)

catkin_package(
CATKIN_DEPENDS 
	geometry_msgs
	roscpp
	rospy 
	serial
	std_msgs
	tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/diffwheel_robot_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

