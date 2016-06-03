cmake_minimum_required(VERSION 2.8.3)
project(foobar)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
)

include_directories(include $(catkin_INCLUDE_DIRS))
add_executable(talker2 src/talker2.cpp)
target_link_libraries(talker2 ${catkin_LIBRARIES})
add_dependencies(talker2 beginner_tutorials_generate_messages_cpp)

add_executable(listener2 src/listener2.cpp)
target_link_libraries(listener2 ${catkin_LIBRARIES})
add_dependencies(listener2 beginner_tutorials_generate_messages_cpp)



