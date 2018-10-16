cmake_minimum_required(VERSION 2.8.3)
project(move_straight)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	amcl
	)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	)
include_directories(include ${catkin_LIBRARIES})

add_executable(MoveStraightLine src/MoveStraightLine.cpp)
target_link_libraries(MoveStraightLine ${catkin_LIBRARIES})

