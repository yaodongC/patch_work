cmake_minimum_required(VERSION 2.8.3)
project(patchwork)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(Boost_LIBRARY_DIR_RELEASE /usr/lib/aarch64-linux-gnu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        )

find_package(Boost REQUIRED)

find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
)



add_message_files(
        FILES
        node.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

# find_package(Boost REQUIRED)
# find_package(PCL 1.8 REQUIRED)
# find_package(Boost 1.65.1 COMPONENTS system filesystem REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(ros_node nodes/ros_node.cpp)
target_link_libraries(ros_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ros_node patchwork_generate_messages_cpp)

