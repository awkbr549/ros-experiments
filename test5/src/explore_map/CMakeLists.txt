cmake_minimum_required(VERSION 2.8.3)
project(explore_map)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
	)
include_directories(include ${catkin_LIBRARIES})

add_executable(ExploreMap src/ExploreMap.cpp)
target_link_libraries(ExploreMap ${catkin_LIBRARIES})

