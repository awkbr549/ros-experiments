cmake_minimum_required(VERSION 2.8.3)
project(set_amclinitpose)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	)
include_directories(include ${catkin_LIBRARIES})

add_executable(SetInitPoseAMCL src/SetInitPoseAMCL.cpp)
target_link_libraries(SetInitPoseAMCL ${catkin_LIBRARIES})

