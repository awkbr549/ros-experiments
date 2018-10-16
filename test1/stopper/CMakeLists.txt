cmake_minimum_required(VERSION 2.8.3)
project(stopper)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	stage_ros
	)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	)
include_directories(include ${catkin_LIBRARIES})

add_executable(AvoidObst_Turtle src/AvoidObst_Turtle.cpp)
target_link_libraries(AvoidObst_Turtle ${catkin_LIBRARIES})

