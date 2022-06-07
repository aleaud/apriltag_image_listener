cmake_minimum_required(VERSION 2.8.3)
project(apriltag_image_listener)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	apriltag_ros
	geometry_msgs
	nav_msgs
	tf
	message_generation
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
)

catkin_package( 
	CATKIN_DEPENDS 
	 roscpp
	 message_runtime
	 sensor_msgs
	 geometry_msgs
)


find_package(apriltag REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

include_directories(src ${apriltag_INCLUDE_DIRS})
include_directories(src ${catkin_INCLUDE_DIRS})

add_executable(apriltag_image_listener_node src/apriltag_image_listener_node.cpp)
target_link_libraries(apriltag_image_listener_node ${apriltag_LIBS} ${catkin_LIBRARIES})
