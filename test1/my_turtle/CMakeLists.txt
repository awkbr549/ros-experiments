cmake_minimum_required(VERSION 2.8.3)
project(my_turtle)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_LIBRARIES})

add_executable(Move_Turtle src/Move_Turtle.cpp)
target_link_libraries(Move_Turtle ${catkin_LIBRARIES})
#add_dependencies(Move_Turtle beginner_tutorials_generate_messages_cpp)

