cmake_minimum_required(VERSION 2.8.3)
project(send_goals)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	actionlib
	geometry_msgs
	move_base_msgs
	)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	)
include_directories(include ${catkin_LIBRARIES})

add_executable(giveWaypoints src/giveWaypoints.cpp)
target_link_libraries(giveWaypoints ${catkin_LIBRARIES})

